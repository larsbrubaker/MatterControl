﻿/*
Copyright (c) 2016, Lars Brubaker
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
using MatterHackers.Agg.PlatformAbstract;
using MatterHackers.Agg.UI;
using MatterHackers.Localizations;
using MatterHackers.MatterControl;
using MatterHackers.MatterControl.CustomWidgets;
using MatterHackers.Agg;
using System.Collections.Generic;
using MatterHackers.MatterControl.SlicerConfiguration;

namespace MatterHackers.MatterControl
{
	public class CopyGuestProfilesToUser : WizardPage
	{
		static string importMessage = "It's time to upload your existing printers to your MatterHackers account. Once uploaded, these printers will be available every time you log into MatterControl.".Localize();

		List<CheckBox> checkBoxes = new List<CheckBox>();

		public CopyGuestProfilesToUser(Action afterProfilesImported)
		: base("Cancel", "Select Printers to Sync")
		{
			var scrollWindow = new ScrollableWidget()
			{
				AutoScroll = true,
				HAnchor = HAnchor.ParentLeftRight,
				VAnchor = VAnchor.ParentBottomTop,
			};
			scrollWindow.ScrollArea.HAnchor = HAnchor.ParentLeftRight;
			contentRow.AddChild(scrollWindow);

			var container = new FlowLayoutWidget(FlowDirection.TopToBottom)
			{
				HAnchor = HAnchor.ParentLeftRight,
			};
			scrollWindow.AddChild(container);

			container.AddChild(new WrappedTextWidget(importMessage, 10, textColor: ActiveTheme.Instance.PrimaryTextColor));

			var byCheckbox = new Dictionary<CheckBox, PrinterInfo>();

			var guestProfileManager = ProfileManager.LoadGuestDB();
			if (guestProfileManager?.Profiles.Count > 0)
			{
				container.AddChild(new TextWidget("Printers to Sync:".Localize())
				{
					TextColor = ActiveTheme.Instance.PrimaryTextColor,
					Margin = new BorderDouble(0, 3, 0, 15),
				});

				foreach (var printerInfo in guestProfileManager.Profiles)
				{
					var checkBox = new CheckBox(printerInfo.Name)
					{
						TextColor = ActiveTheme.Instance.PrimaryTextColor,
						Margin = new BorderDouble(5, 0, 0, 0),
						HAnchor = HAnchor.ParentLeft,
						Checked = true,
					};
					checkBoxes.Add(checkBox);
					container.AddChild(checkBox);

					byCheckbox[checkBox] = printerInfo;
				}
			}

			var uploadButton = textImageButtonFactory.Generate("Sync".Localize());
			uploadButton.Click += (s, e) =>
			{
				// do the import
				foreach(var checkBox in checkBoxes)
				{
					if (checkBox.Checked)
					{
						// import the printer
						var printerInfo = byCheckbox[checkBox];

						ProfileManager.Instance.Profiles.Add(printerInfo);
						guestProfileManager.Profiles.Remove(printerInfo);
					}
				}

				guestProfileManager.Save();

				// close the window
				UiThread.RunOnIdle(() =>
				{
					WizardWindow.Close();

					// Call back into the original source
					afterProfilesImported();
				});
			};

			uploadButton.Visible = true;
			cancelButton.Visible = true;

			cancelButton.Click += (s, e) => UiThread.RunOnIdle(WizardWindow.Close);

			//Add buttons to buttonContainer
			footerRow.AddChild(uploadButton);
			footerRow.AddChild(new HorizontalSpacer());
			footerRow.AddChild(cancelButton);

			footerRow.Visible = true;
		}
	}
}