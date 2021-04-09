﻿/*
Copyright (c) 2018, Lars Brubaker, John Lewin
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the FreeBSD Project.
*/

using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Threading.Tasks;
using MatterControlLib;
using MatterHackers.Agg;
using MatterHackers.Agg.Image;
using MatterHackers.Agg.Platform;
using MatterHackers.Agg.UI;
using MatterHackers.DataConverters3D;
using MatterHackers.Localizations;
using MatterHackers.MatterControl.CustomWidgets;
using MatterHackers.MatterControl.Library;
using MatterHackers.MatterControl.PartPreviewWindow.PlusTab;
using MatterHackers.MatterControl.PrintLibrary;
using MatterHackers.MatterControl.SlicerConfiguration;
using MatterHackers.VectorMath;

namespace MatterHackers.MatterControl.PartPreviewWindow
{
	public class MainViewWidget : FlowLayoutWidget
	{
		private EventHandler unregisterEvents;
		private ChromeTabs tabControl;

		private int partCount = 0;
		private ThemeConfig theme;
		private Toolbar statusBar;
		private FlowLayoutWidget tasksContainer;
		private GuiWidget stretchStatusPanel;
		private LinkLabel updateAvailableButton;

		public MainViewWidget(ThemeConfig theme)
			: base(FlowDirection.TopToBottom)
		{
			this.AnchorAll();
			this.theme = theme;
			this.Name = "PartPreviewContent";
			this.BackgroundColor = theme.BackgroundColor;

			// Push TouchScreenMode into GuiWidget
			GuiWidget.TouchScreenMode = UserSettings.Instance.IsTouchScreen;

			AddStandardUi(theme);
			ApplicationController.Instance.WorkspacesChanged += Workspaces_Changed;
			ApplicationController.Instance.Tasks.TasksChanged += Tasks_TasksChanged;
			tabControl.ActiveTabChanged += TabControl_ActiveTabChanged;

			// Register listeners
			PrinterSettings.AnyPrinterSettingChanged += Printer_SettingChanged;

			ApplicationController.Instance.ShellFileOpened += this.Instance_OpenNewFile;

			ApplicationController.Instance.MainView = this;
		}

		public override void OnLoad(EventArgs args)
		{
			if (Application.MiniTouchScreen.Enabled)
			{
				var miniTouchScreen = new SystemWindow(800, 480);

				miniTouchScreen.ShowAsSystemWindow();
			}

			base.OnLoad(args);
		}

		private void AddStandardUi(ThemeConfig theme)
		{
			var extensionArea = new LeftClipFlowLayoutWidget()
			{
				BackgroundColor = theme.TabBarBackground,
				VAnchor = VAnchor.Stretch,
				Padding = new BorderDouble(left: 8)
			};

			SearchPanel searchPanel = null;

			bool searchPanelOpenOnMouseDown = false;

			var searchButton = theme.CreateSearchButton();
			searchButton.Name = "App Search Button";
			searchButton.MouseDown += (s, e) =>
			{
				searchPanelOpenOnMouseDown = searchPanel != null;
			};

			searchButton.Click += SearchButton_Click;
			extensionArea.AddChild(searchButton);

			async void SearchButton_Click(object sender, EventArgs e)
			{
				if (searchPanel == null && !searchPanelOpenOnMouseDown)
				{
					void ShowSearchPanel()
					{
						searchPanel = new SearchPanel(this.TabControl, searchButton, theme);
						searchPanel.Closed += SearchPanel_Closed;

						var systemWindow = this.Parents<SystemWindow>().FirstOrDefault();
						systemWindow.ShowRightSplitPopup(
							new MatePoint(searchButton),
							new MatePoint(searchPanel),
							borderWidth: 0);
					}

					if (HelpIndex.IndexExists)
					{
						ShowSearchPanel();
					}
					else
					{
						searchButton.Enabled = false;

						try
						{
							// Show popover
							var popover = new Popover(ArrowDirection.Up, 7, 5, 0)
							{
								TagColor = theme.AccentMimimalOverlay
							};

							popover.AddChild(new TextWidget("Preparing help".Localize() + "...", pointSize: theme.DefaultFontSize - 1, textColor: theme.TextColor));

							popover.ArrowOffset = (int)(popover.Width - (searchButton.Width / 2));

							this.Parents<SystemWindow>().FirstOrDefault().ShowPopover(
								new MatePoint(searchButton)
								{
									Mate = new MateOptions(MateEdge.Right, MateEdge.Bottom),
									AltMate = new MateOptions(MateEdge.Right, MateEdge.Bottom),
									Offset = new RectangleDouble(12, 0, 12, 0)
								},
								new MatePoint(popover)
								{
									Mate = new MateOptions(MateEdge.Right, MateEdge.Top),
									AltMate = new MateOptions(MateEdge.Left, MateEdge.Bottom)
								});

							await Task.Run(async () =>
							{
								// Start index generation
								await HelpIndex.RebuildIndex();

								UiThread.RunOnIdle(() =>
								{
									// Close popover
									popover.Close();

									// Continue to original task
									ShowSearchPanel();
								});
							});
						}
						catch
						{
						}

						searchButton.Enabled = true;
					}
				}
				else
				{
					searchPanel?.CloseOnIdle();
					searchPanelOpenOnMouseDown = false;
				}
			}

			void SearchPanel_Closed(object sender, EventArgs e)
			{
				// Unregister
				searchPanel.Closed -= SearchPanel_Closed;

				// Release
				searchPanel = null;
			}

			tabControl = new ChromeTabs(extensionArea, theme)
			{
				VAnchor = VAnchor.Stretch,
				HAnchor = HAnchor.Stretch,
				BackgroundColor = theme.BackgroundColor,
				BorderColor = theme.MinimalShade,
				Border = new BorderDouble(left: 1),
			};

			tabControl.PlusClicked += (s, e) => UiThread.RunOnIdle(() =>
			{
				this.CreatePartTab().ConfigureAwait(false);
			});

			// Force the ActionArea to be as high as ButtonHeight
			tabControl.TabBar.ActionArea.MinimumSize = new Vector2(0, theme.ButtonHeight);
			tabControl.TabBar.BackgroundColor = theme.TabBarBackground;
			tabControl.TabBar.BorderColor = theme.BackgroundColor;

			// Force common padding into top region
			tabControl.TabBar.Padding = theme.TabbarPadding.Clone(top: theme.TabbarPadding.Top * 2, bottom: 0);

			if (Application.EnableNetworkTraffic)
			{
				// add in the update available button
				updateAvailableButton = new LinkLabel("Update Available".Localize(), theme)
				{
					Visible = false,
					Name = "Update Available Link",
					ToolTipText = "There is a new update available for download".Localize(),
					VAnchor = VAnchor.Center,
					Margin = new BorderDouble(10, 0),
					TextColor = theme.PrimaryAccentColor
				};

				// Register listeners
				UserSettings.Instance.SettingChanged += SetLinkButtonsVisibility;

				SetLinkButtonsVisibility(this, null);

				updateAvailableButton.Click += (s, e) =>
				{
					UpdateControlData.Instance.CheckForUpdate();
					DialogWindow.Show<CheckForUpdatesPage>();
				};

				tabControl.TabBar.ActionArea.AddChild(updateAvailableButton);

				UpdateControlData.Instance.UpdateStatusChanged.RegisterEvent((s, e) =>
				{
					SetLinkButtonsVisibility(s, new StringEventArgs("Unknown"));
				}, ref unregisterEvents);
			}

			this.AddChild(tabControl);

			ApplicationController.Instance.NotifyPrintersTabRightElement(extensionArea);

			ChromeTab tab = null;

			// Upgrade tab
			if (!ApplicationController.Instance.IsMatterControlPro())
			{
				tabControl.AddTab(
					tab = new ChromeTab("Upgrade", "Upgrade".Localize(), tabControl, new UpgradeToProTabPage(theme), theme, hasClose: false)
					{
						MinimumSize = new Vector2(0, theme.TabButtonHeight),
						Name = "Upgrade",
						Padding = new BorderDouble(15, 0),
					});

				tab.AfterDraw += (s, e) =>
				{
					e.Graphics2D.Circle(tab.LocalBounds.Right - 25 * DeviceScale,
						tab.LocalBounds.Bottom + tab.Height / 2 - 1 * DeviceScale,
						5 * DeviceScale,
						theme.PrimaryAccentColor);
				};
			}
			else
			{
				// Store tab
				tabControl.AddTab(
					tab = new ChromeTab("Store", "Store".Localize(), tabControl, new StoreTabPage(theme), theme, hasClose: false)
					{
						MinimumSize = new Vector2(0, theme.TabButtonHeight),
						Name = "Store Tab",
						Padding = new BorderDouble(15, 0),
					});
			}

			EnableReduceWidth(tab, theme);

			// Library tab
			var libraryWidget = new LibraryWidget(this, theme)
			{
				BackgroundColor = theme.BackgroundColor
			};

			tabControl.AddTab(
				tab = new ChromeTab("Library", "Library".Localize(), tabControl, libraryWidget, theme, hasClose: false)
				{
					MinimumSize = new Vector2(0, theme.TabButtonHeight),
					Name = "Library Tab",
					Padding = new BorderDouble(15, 0),
				});
			EnableReduceWidth(tab, theme);

			// Hardware tab
			tabControl.AddTab(
				tab = new ChromeTab(
					"Hardware",
					"Hardware".Localize(),
					tabControl,
					new HardwareTabPage(theme)
					{
						BackgroundColor = theme.BackgroundColor
					},
					theme,
					hasClose: false)
				{
					MinimumSize = new Vector2(0, theme.TabButtonHeight),
					Name = "Hardware Tab",
					Padding = new BorderDouble(15, 0),
				});
			EnableReduceWidth(tab, theme);

			SetInitialTab();

			var brandMenu = new BrandMenuButton(theme)
			{
				HAnchor = HAnchor.Fit,
				VAnchor = VAnchor.Fit,
				BackgroundColor = theme.TabBarBackground,
				Padding = theme.TabbarPadding.Clone(right: theme.DefaultContainerPadding)
			};

			tabControl.TabBar.ActionArea.AddChild(brandMenu, 0);

			tabControl.TabBar.ActionArea.VAnchor = VAnchor.Absolute;
			tabControl.TabBar.ActionArea.Height = brandMenu.Height;
			tabControl.FirstMovableTab = tabControl.AllTabs.Count();

			// Restore active workspace tabs
			foreach (var workspace in ApplicationController.Instance.Workspaces)
			{
				ChromeTab newTab;

				// Create and switch to new printer tab
				if (workspace.Printer?.Settings.PrinterSelected == true)
				{
					newTab = this.CreatePrinterTab(workspace, theme);
				}
				else
				{
					newTab = this.CreatePartTab(workspace);
				}

				if (newTab.Key == ApplicationController.Instance.MainTabKey)
				{
					tabControl.ActiveTab = newTab;
				}
			}

			statusBar = new Toolbar(theme.TabbarPadding)
			{
				HAnchor = HAnchor.Stretch,
				VAnchor = VAnchor.Absolute,
				Padding = 1,
				Height = 22 * GuiWidget.DeviceScale,
				BackgroundColor = theme.BackgroundColor,
				Border = new BorderDouble(top: 1),
				BorderColor = theme.BorderColor20,
			};
			this.AddChild(statusBar);

			statusBar.ActionArea.VAnchor = VAnchor.Stretch;

			tasksContainer = new FlowLayoutWidget(FlowDirection.LeftToRight)
			{
				HAnchor = HAnchor.Fit,
				VAnchor = VAnchor.Stretch,
				BackgroundColor = theme.MinimalShade,
				Name = "runningTasksPanel"
			};
			statusBar.AddChild(tasksContainer);

			stretchStatusPanel = new GuiWidget()
			{
				HAnchor = HAnchor.Stretch,
				VAnchor = VAnchor.Stretch,
				Padding = new BorderDouble(right: 3),
				Margin = new BorderDouble(right: 2, top: 1, bottom: 1),
				Border = new BorderDouble(1),
				BackgroundColor = theme.MinimalShade.WithAlpha(10),
				BorderColor = theme.SlightShade,
				Width = 200 * GuiWidget.DeviceScale
			};
			statusBar.AddChild(stretchStatusPanel);

			var panelBackgroundColor = theme.MinimalShade.WithAlpha(10);

			statusBar.AddChild(this.CreateThemeStatusPanel(theme, panelBackgroundColor));

			statusBar.AddChild(this.CreateNetworkStatusPanel(theme));

			this.RenderRunningTasks(theme, ApplicationController.Instance.Tasks);
		}

		private void SetInitialTab()
		{
			// Initial tab selection - workspace load will reset if applicable
			string tabKey = ApplicationController.Instance.MainTabKey;

			if (string.IsNullOrEmpty(tabKey)
				|| !tabControl.AllTabs.Any(t => t.Key == tabKey))
			{
				tabKey = "Hardware";
			}

			tabControl.SelectedTabKey = tabKey;
		}

		private async void Instance_OpenNewFile(object sender, string filePath)
		{
			var history = ApplicationController.Instance.Library.PlatingHistory;

			var workspace = new PartWorkspace(new BedConfig(history))
			{
				Name = Path.GetFileName(filePath),
			};

			ApplicationController.Instance.Workspaces.Add(workspace);

			var scene = new Object3D();
			scene.Children.Add(
				new Object3D
				{
					MeshPath = filePath,
					Name = Path.GetFileName(filePath)
				});

			await workspace.SceneContext.LoadContent(
				new EditContext()
				{
					ContentStore = ApplicationController.Instance.Library.PlatingHistory,
					SourceItem = new InMemoryLibraryItem(scene)
				});

			ApplicationController.Instance.Workspaces.Add(workspace);

			var newTab = CreatePartTab(workspace);
			tabControl.ActiveTab = newTab;
		}

		private void TabControl_ActiveTabChanged(object sender, EventArgs e)
		{
			if (this.tabControl.ActiveTab?.TabContent is PartTabPage tabPage)
			{
				var dragDropData = ApplicationController.Instance.DragDropData;

				// Set reference on tab change
				dragDropData.View3DWidget = tabPage.view3DWidget;
				dragDropData.SceneContext = tabPage.sceneContext;
			}

			ApplicationController.Instance.MainTabKey = tabControl.SelectedTabKey;
		}

		private void Tasks_TasksChanged(object sender, EventArgs e)
		{
			this.RenderRunningTasks(theme, ApplicationController.Instance.Tasks);
		}

		public override void OnDraw(Graphics2D graphics2D)
		{
			base.OnDraw(graphics2D);

			// AggContext.DefaultFont.ShowDebugInfo(graphics2D);
		}

		private void ShowUpdateAvailableAnimation()
		{
			double displayTime = 2;
			double pulseTime = 1;
			double totalSeconds = 0;

			var textWidgets = updateAvailableButton.Descendants<TextWidget>().Where((w) => w.Visible == true).ToArray();
			Color startColor = theme.PrimaryAccentColor;

			// Show a highlight on the button as the user did not click it
			var flashBackground = new Animation()
			{
				DrawTarget = updateAvailableButton,
				FramesPerSecond = 10,
			};

			flashBackground.Update += (s1, updateEvent) =>
			{
				totalSeconds += updateEvent.SecondsPassed;
				if (totalSeconds < displayTime)
				{
					double blend = AttentionGetter.GetFadeInOutPulseRatio(totalSeconds, pulseTime);
					var color = new Color(startColor, (int)((1 - blend) * 255));
					foreach (var textWidget in textWidgets)
					{
						textWidget.TextColor = color;
					}
				}
				else
				{
					foreach (var textWidget in textWidgets)
					{
						textWidget.TextColor = startColor;
					}

					flashBackground.Stop();
				}
			};

			flashBackground.Start();
		}

		private void SetLinkButtonsVisibility(object s, StringEventArgs e)
		{
			if (UpdateControlData.Instance.UpdateStatus == UpdateControlData.UpdateStatusStates.UpdateAvailable)
			{
				if (!updateAvailableButton.Visible)
				{
					updateAvailableButton.Visible = true;

					this.ShowUpdateAvailableAnimation();
				}
			}
			else
			{
				updateAvailableButton.Visible = false;
			}
		}

		private void Workspaces_Changed(object sender, WorkspacesChangedEventArgs e)
		{
			var workspace = e.Workspace;
			var activePrinter = workspace.Printer;

			if (e.Operation == WorkspacesChangedEventArgs.OperationType.Add
				|| e.Operation == WorkspacesChangedEventArgs.OperationType.Restore)
			{
				// Create printer or part tab
				bool isPrinter = activePrinter?.Settings.PrinterSelected == true;
				ChromeTab newTab = isPrinter ? CreatePrinterTab(workspace, theme) : CreatePartTab(workspace);

				if (e.Operation == WorkspacesChangedEventArgs.OperationType.Add)
				{
					ApplicationController.Instance.MainTabKey = newTab.Key;
				}

				// Activate tab with previously active key
				if (newTab.Key == ApplicationController.Instance.MainTabKey)
				{
					tabControl.ActiveTab = newTab;
				}
			}
			else
			{
				// Close existing printer tabs
				if (tabControl.AllTabs.FirstOrDefault(t => t.TabContent is PrinterTabPage printerTab
						&& printerTab.Printer.Settings.ID == activePrinter.Settings.ID) is ITab tab
					&& tab.TabContent is PrinterTabPage printerPage)
				{
					tabControl.CloseTab(tab);
				}
			}
		}

		private GuiWidget CreateNetworkStatusPanel(ThemeConfig theme)
		{
			var networkStatus = new GuiWidget()
			{
				HAnchor = HAnchor.Absolute,
				VAnchor = VAnchor.Stretch,
				Padding = new BorderDouble(right: 3),
				Margin = new BorderDouble(right: 2, top: 1, bottom: 1),
				Border = new BorderDouble(1),
				BackgroundColor = theme.MinimalShade.WithAlpha(10),
				BorderColor = theme.SlightShade,
				Width = 120 * GuiWidget.DeviceScale
			};
			if (ApplicationController.ServicesStatusType != null)
			{
				var instance = Activator.CreateInstance(ApplicationController.ServicesStatusType);
				if (instance is GuiWidget guiWidget)
				{
					guiWidget.HAnchor = HAnchor.Stretch;
					guiWidget.VAnchor = VAnchor.Stretch;
					networkStatus.AddChild(guiWidget);
				}
			}

			return networkStatus;
		}

		private GuiWidget CreateThemeStatusPanel(ThemeConfig theme, Color panelBackgroundColor)
		{
			var themePanel = new GuiWidget()
			{
				HAnchor = HAnchor.Fit,
				VAnchor = VAnchor.Stretch,
				Margin = new BorderDouble(right: 2, top: 1, bottom: 1),
				Border = new BorderDouble(1),
				BackgroundColor = panelBackgroundColor,
				BorderColor = theme.SlightShade,
				Cursor = Cursors.Hand,
				ToolTipText = "Theme".Localize(),
				Name = "Theme Select Button"
			};

			themePanel.AddChild(
				new ImageWidget(StaticData.Instance.LoadIcon("theme.png", 16, 16, theme.InvertIcons), false)
				{
					HAnchor = HAnchor.Center | HAnchor.Absolute,
					VAnchor = VAnchor.Center | VAnchor.Absolute,
					Margin = new BorderDouble(5, 0),
					Selectable = false
				});

			themePanel.Click += (s, e) =>
			{
				themePanel.BackgroundColor = theme.DropList.Open.BackgroundColor;

				var menuTheme = AppContext.MenuTheme;
				var widget = new GuiWidget()
				{
					HAnchor = HAnchor.Absolute,
					VAnchor = VAnchor.Fit,
					Width = 650 * GuiWidget.DeviceScale,
					Border = 1,
					BorderColor = theme.DropList.Open.BackgroundColor,
					// Padding = theme.DefaultContainerPadding,
					BackgroundColor = menuTheme.BackgroundColor
				};

				var section = ApplicationSettingsPage.CreateThemePanel(menuTheme);
				widget.AddChild(section);

				var systemWindow = this.Parents<SystemWindow>().FirstOrDefault();
				systemWindow.ShowPopup(
					new MatePoint(themePanel)
					{
						Mate = new MateOptions(MateEdge.Right, MateEdge.Top),
						AltMate = new MateOptions(MateEdge.Right, MateEdge.Top)
					},
					new MatePoint(widget)
					{
						Mate = new MateOptions(MateEdge.Right, MateEdge.Bottom),
						AltMate = new MateOptions(MateEdge.Right, MateEdge.Bottom)
					});
			};
			return themePanel;
		}

		public ChromeTabs TabControl => tabControl;

		private static int debugPrinterTabIndex = 0;

		private ChromeTab CreatePrinterTab(PartWorkspace workspace, ThemeConfig theme)
		{
			var printer = workspace.Printer;
			// Printer page is in fixed position
			var tab1 = tabControl.AllTabs.FirstOrDefault();

			var printerTabPage = tab1?.TabContent as PrinterTabPage;
			if (printerTabPage == null
				|| printerTabPage.Printer != printer)
			{
				// TODO - call save before remove
				// printerTabPage.sceneContext.SaveChanges();

				if (printerTabPage != null)
				{
					tabControl.CloseTab(tab1);
				}

				var printerTab = new ChromeTab(
					printer.Settings.GetValue(SettingsKey.printer_name),
					printer.Settings.GetValue(SettingsKey.printer_name),
					tabControl,
					new PrinterTabPage(workspace, theme, "unused_tab_title"),
					theme,
					tabImageUrl: ApplicationController.Instance.GetFavIconUrl(oemName: printer.Settings.GetValue(SettingsKey.make)))
				{
					Name = $"3D View Tab {debugPrinterTabIndex++}",
				};

				// add a right click menu
				printerTab.Click += (s, e) =>
				{
					if (e.Button == MouseButtons.Right)
					{
						AddRightClickTabMenu(tabControl, printerTab, printer, null, e);
					}
				};

				EnableReduceWidth(printerTab, theme);

				void Tab_CloseClicked(object sender, EventArgs args)
				{
					ApplicationController.Instance.ClosePrinter(printer);
				}

				void Widget_Closed(object sender, EventArgs args)
				{
					// Unregister listeners
					printerTab.CloseClicked -= Tab_CloseClicked;
					printerTab.Closed -= Widget_Closed;
					printer.Settings.SettingChanged -= Printer_SettingChanged;
				}

				// Register listeners
				printer.Settings.SettingChanged += Printer_SettingChanged;
				printerTab.CloseClicked += Tab_CloseClicked;
				printerTab.Closed += Widget_Closed;

				// Add printer tab
				tabControl.AddTab(printerTab);

				return printerTab;
			}
			else if (tab1 != null)
			{
				tabControl.ActiveTab = tab1;
				return tab1 as ChromeTab;
			}

			return null;
		}

		private void AddRightClickTabMenu(ChromeTabs tabs, ChromeTab printerTab, PrinterConfig printer, PartWorkspace workspace, MouseEventArgs mouseEvent)
		{
			var menuTheme = ApplicationController.Instance.MenuTheme;
			var popupMenu = new PopupMenu(menuTheme);

			var renameMenuItem = popupMenu.CreateMenuItem("Rename".Localize());
			renameMenuItem.Click += (s, e) =>
			{
				if (workspace != null)
				{
					workspace.SceneContext?.EditContext?.SourceItem?.Rename();
				}
				else if (printer != null)
				{
					DialogWindow.Show(
						new InputBoxPage(
							"Rename Item".Localize(),
							"Name".Localize(),
							printer.Settings.GetValue(SettingsKey.printer_name),
							"Enter New Name Here".Localize(),
							"Rename".Localize(),
							(newName) =>
							{
								printer.Settings.SetValue(SettingsKey.printer_name, newName);
							}));
				}
			};


			var moveButtons = new FlowLayoutWidget();

			var textWidget = new TextWidget("Move Tab", pointSize: theme.DefaultFontSize, textColor: theme.TextColor)
			{
				Margin = PopupMenu.MenuPadding.Clone(PopupMenu.MenuPadding.Left - 5, right: 5),
				VAnchor = VAnchor.Center,
			};
			moveButtons.AddChild(textWidget);
			var buttonSize = 24 * DeviceScale;
			var moveLeftButton = new IconButton(StaticData.Instance.LoadIcon("fa-angle-right_12.png", 14, 14, theme.InvertIcons).MirrorX(), theme)
			{
				Width = buttonSize,
				Height = buttonSize,
				Margin = new BorderDouble(3, 0),
				HoverColor = theme.AccentMimimalOverlay,
				VAnchor = VAnchor.Center,
				Enabled = tabs.GetTabIndex(printerTab) > tabs.FirstMovableTab,
			};
			moveLeftButton.Click += (s, e) =>
			{
				tabs.MoveTabLeft(printerTab);
				popupMenu.Unfocus();
			};
			moveButtons.AddChild(moveLeftButton);

			var moveRightButton = new IconButton(StaticData.Instance.LoadIcon("fa-angle-right_12.png", 14, 14, theme.InvertIcons), theme)
			{
				Width = buttonSize,
				Height = buttonSize,
				Margin = new BorderDouble(3, 0),
				HoverColor = theme.AccentMimimalOverlay,
				VAnchor = VAnchor.Center,
				Enabled = printerTab.NextTab != null,
			};

			moveRightButton.Click += (s, e) =>
			{
				tabs.MoveTabRight(printerTab);
				popupMenu.Unfocus();
			};
			moveButtons.AddChild(moveRightButton);

			popupMenu.AddChild(moveButtons);

			popupMenu.ShowMenu(printerTab, mouseEvent);
		}

		public async Task<ChromeTab> CreatePartTab()
		{
			var history = ApplicationController.Instance.Library.PlatingHistory;

			var workspace = new PartWorkspace(new BedConfig(history))
			{
				Name = "New Design".Localize() + (partCount == 0 ? "" : $" ({partCount})"),
			};

			await workspace.SceneContext.LoadContent(
				new EditContext()
				{
					ContentStore = ApplicationController.Instance.Library.PlatingHistory,
					SourceItem = history.NewPlatingItem(workspace.SceneContext.Scene)
				});

			ApplicationController.Instance.Workspaces.Add(workspace);

			var newTab = CreatePartTab(workspace);
			tabControl.ActiveTab = newTab;

			return newTab;
		}

		public ChromeTab CreatePartTab(PartWorkspace workspace)
		{
			var partTab = new ChromeTab(
				workspace.Name,
				workspace.Name,
				tabControl,
				new PartTabPage(workspace, theme, ""),
				theme,
				StaticData.Instance.LoadIcon("cube.png", 16, 16, theme.InvertIcons))
			{
				Name = "newPart" + tabControl.AllTabs.Count(),
			};

			EnableReduceWidth(partTab, theme);

			tabControl.AddTab(partTab);

			void Tab_CloseClicked(object sender, EventArgs args)
			{
				ApplicationController.Instance.Workspaces.Remove(workspace);
			}

			// add a right click menu
			partTab.Click += (s, e) =>
			{
				if (e.Button == MouseButtons.Right)
				{
					AddRightClickTabMenu(tabControl, partTab, null, workspace, e);
				}
			};

			void Widget_Closed(object sender, EventArgs args)
			{
				partTab.CloseClicked -= Tab_CloseClicked;
				partTab.Closed -= Widget_Closed;
			}

			partTab.CloseClicked += Tab_CloseClicked;
			partTab.Closed += Widget_Closed;

			return partTab;
		}

		private static void EnableReduceWidth(ChromeTab partTab, ThemeConfig theme)
		{
			var scale = GuiWidget.DeviceScale;
			partTab.MinimumSize = new Vector2(80 * scale, theme.TabButtonHeight);

			var textWidget = partTab.Descendants<TextWidget>().First();
			var tabPill = partTab.Descendants<SimpleTab.TabPill>().First();
			tabPill.HAnchor = HAnchor.Stretch;
			var closeBox = partTab.Descendants<ImageWidget>().FirstOrDefault();
			if (closeBox != null)
			{
				var tabPillMarign = tabPill.Margin;
				tabPill.Margin = new BorderDouble(tabPillMarign.Left, tabPillMarign.Bottom, tabPillMarign.Right + 10, tabPillMarign.Top);
			}

			UpadetMaxWidth();

			// delay this for an update so that the layout of the text widget has happened and its size has been updated.
			textWidget.TextChanged += (s, e) => UiThread.RunOnIdle(UpadetMaxWidth);

			void UpadetMaxWidth()
			{
				// the text
				var width = textWidget.Width;
				// the tab pill
				width += (tabPill.Margin.Width + tabPill.Padding.Width) * scale;
				if (closeBox != null)
				{
					// the close box
					width += closeBox.Width;
				}
				else
				{
					width += 32 * scale;
				}

				partTab.MaximumSize = new Vector2(width, partTab.MaximumSize.Y);
				partTab.Width -= 1;

				// wait for this size change to take effect and update the tool tip
				partTab.BoundsChanged += (s, e) =>
				{
					if (partTab.Width < partTab.MaximumSize.X)
					{
						partTab.ToolTipText = textWidget.Text;
					}
					else
					{
						partTab.ToolTipText = "";
					}
				};
			}

			partTab.HAnchor = HAnchor.Stretch;
		}

		public override void OnClosed(EventArgs e)
		{
			// Unregister listeners
			PrinterSettings.AnyPrinterSettingChanged -= Printer_SettingChanged;
			UserSettings.Instance.SettingChanged -= SetLinkButtonsVisibility;
			ApplicationController.Instance.WorkspacesChanged -= Workspaces_Changed;
			ApplicationController.Instance.Tasks.TasksChanged -= Tasks_TasksChanged;
			ApplicationController.Instance.ShellFileOpened -= Instance_OpenNewFile;
			if (tabControl != null)
			{
				tabControl.ActiveTabChanged -= TabControl_ActiveTabChanged;
			}

			unregisterEvents?.Invoke(this, null);
			base.OnClosed(e);
		}

#if DEBUG
		~MainViewWidget()
		{
			Console.WriteLine();
		}
#endif

		private void Printer_SettingChanged(object s, StringEventArgs stringEvent)
		{
			if (s is PrinterSettings printerSettings
				&& stringEvent?.Data == SettingsKey.printer_name)
			{
				// Try to find a printer tab for the given printer
				var printerTab = tabControl.AllTabs.FirstOrDefault(t => t.TabContent is PrinterTabPage printerPage && printerPage.Printer.Settings.ID == printerSettings.ID) as ChromeTab;
				if (printerTab != null)
				{
					printerTab.Title = printerSettings.GetValue(SettingsKey.printer_name);
					printerTab.ToolTipText = printerTab.Title;
				}
			}
		}

		private void RenderRunningTasks(ThemeConfig theme, RunningTasksConfig tasks)
		{
			var rows = tasksContainer.Children.OfType<RunningTaskStatusPanel>().ToList();
			var displayedTasks = new HashSet<RunningTaskDetails>(rows.Select(taskRow => taskRow.taskDetails));
			var runningTasks = tasks.RunningTasks;

			// Remove expired items
			foreach (var row in rows)
			{
				if (!runningTasks.Contains(row.taskDetails))
				{
					row.Close();
				}
			}

			var progressBackgroundColor = new Color(theme.AccentMimimalOverlay, 35);

			// Add new items
			foreach (var taskItem in tasks.RunningTasks.Where(t => !displayedTasks.Contains(t)))
			{
				// TODO: find out how we are getting a null task item in the list
				if (taskItem == null)
				{
					continue;
				}

				var runningTaskPanel = new RunningTaskStatusPanel("", taskItem, theme)
				{
					HAnchor = HAnchor.Absolute,
					VAnchor = VAnchor.Stretch,
					Margin = new BorderDouble(right: 2, top: 1, bottom: 1),
					Border = new BorderDouble(1),
					BorderColor = theme.SlightShade,
					ProgressBackgroundColor = progressBackgroundColor,
					Width = 200 * GuiWidget.DeviceScale
				};

				tasksContainer.AddChild(runningTaskPanel);
			}

			tasksContainer.Invalidate();
		}
	}
}