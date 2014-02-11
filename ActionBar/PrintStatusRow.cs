﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Globalization;
using System.Diagnostics;
using System.Threading;
using System.IO;

using MatterHackers.Agg.Image;
using MatterHackers.Agg;
using MatterHackers.Agg.UI;
using MatterHackers.VectorMath;
using MatterHackers.Agg.VertexSource;
using MatterHackers.MatterControl;
using MatterHackers.MatterControl.DataStorage;
using MatterHackers.MatterControl.PrintQueue;

using MatterHackers.PolygonMesh;

using MatterHackers.Localizations; 

namespace MatterHackers.MatterControl.ActionBar
{
    class PrintStatusRow : ActionRowBase
    {
        Stopwatch timeSinceLastDrawTime = new Stopwatch();
        event EventHandler unregisterEvents;

        TextWidget activePrintName;
        TextWidget activePrintLabel;
        TextWidget activePrintInfo;
        TextInfo textInfo = new CultureInfo("en-US", false).TextInfo;

        TextWidget activePrintStatus;
        string ActivePrintStatusText
        {
            set 
            {
                if (activePrintStatus.Text != value)
                {
                    activePrintStatus.Text = value;
                }
            }
        }

        protected override void Initialize()
        {
            UiThread.RunOnIdle(OnIdle);
            this.Margin = new BorderDouble(6, 3, 6, 6);
        }

        PartThumbnailWidget activePrintPreviewImage;
        void onActivePrintItemChanged(object sender, EventArgs e)
        {
            // first we have to remove any link to an old part (the part currently in the view)
            if (activePrintPreviewImage.PrintItem != null)
            {
                activePrintPreviewImage.PrintItem.SlicingOutputMessage -= PrintItem_SlicingOutputMessage;
            }

            activePrintPreviewImage.PrintItem = PrinterCommunication.Instance.ActivePrintItem;

            // then hook up our new part
            if (activePrintPreviewImage.PrintItem != null)
            {
                activePrintPreviewImage.PrintItem.SlicingOutputMessage += PrintItem_SlicingOutputMessage;
            }

            activePrintPreviewImage.Invalidate();
        }

        void PrintItem_SlicingOutputMessage(object sender, EventArgs e)
        {
            StringEventArgs message = e as StringEventArgs;
            ActivePrintStatusText = message.Data;
        }

        override protected void AddChildElements()
        {            
            activePrintPreviewImage = new PartThumbnailWidget(null, "part_icon_transparent_100x100.png", "building_thumbnail_100x100.png", new Vector2(115, 115));
            activePrintPreviewImage.VAnchor = VAnchor.ParentTop;
            activePrintPreviewImage.Padding = new BorderDouble(0);
            activePrintPreviewImage.HoverBackgroundColor = new RGBA_Bytes();
            activePrintPreviewImage.BorderWidth = 3;

            FlowLayoutWidget temperatureWidgets = new FlowLayoutWidget(FlowDirection.TopToBottom);
            {
                IndicatorWidget extruderTemperatureWidget = new ExtruderTemperatureWidget();
                IndicatorWidget bedTemperatureWidget = new BedTemperatureWidget();

                temperatureWidgets.AddChild(extruderTemperatureWidget);
                temperatureWidgets.AddChild(bedTemperatureWidget);
            }            
            temperatureWidgets.VAnchor = VAnchor.ParentTop;

            FlowLayoutWidget printStatusContainer = getActivePrinterInfo();
            printStatusContainer.VAnchor = VAnchor.ParentTop;

            this.AddChild(activePrintPreviewImage);
            this.AddChild(printStatusContainer);
            this.AddChild(temperatureWidgets);

            UpdatePrintStatus();
            UpdatePrintItemName();
        }

        private FlowLayoutWidget getActivePrinterInfo()
        {
            FlowLayoutWidget container = new FlowLayoutWidget(FlowDirection.TopToBottom);
            container.Margin = new BorderDouble(6, 0,6,3);
            container.HAnchor = HAnchor.ParentLeftRight;
            container.VAnchor |= VAnchor.ParentTop;

            FlowLayoutWidget topRow = new FlowLayoutWidget();
            topRow.Name = "PrintStatusRow.ActivePrinterInfo.TopRow";
            topRow.HAnchor = HAnchor.ParentLeftRight;

			activePrintLabel = getPrintStatusLabel(new LocalizedString("Next Print:").Translated, pointSize: 11);
            activePrintLabel.VAnchor = VAnchor.ParentTop;

            topRow.AddChild(activePrintLabel);

            activePrintName = getPrintStatusLabel("this is the biggest name we will allow", pointSize: 14);
            activePrintName.AutoExpandBoundsToText = false;
            activePrintStatus = getPrintStatusLabel("this is the biggest label we will allow - bigger", pointSize: 11);
            activePrintStatus.AutoExpandBoundsToText = false;
            activePrintStatus.Text = "";
            activePrintStatus.Margin = new BorderDouble(top: 3);

            activePrintInfo = getPrintStatusLabel("", pointSize: 11);
            activePrintInfo.AutoExpandBoundsToText = true;

            PrintActionRow printActionRow = new PrintActionRow();

            container.AddChild(topRow);
            container.AddChild(activePrintName);
            container.AddChild(activePrintStatus);
            //container.AddChild(activePrintInfo);
            container.AddChild(printActionRow);
            container.AddChild(new MessageActionRow());

            return container;
        }

        protected override void AddHandlers()
        {
            PrinterCommunication.Instance.ActivePrintItemChanged.RegisterEvent(onPrintItemChanged, ref unregisterEvents);
            PrinterCommunication.Instance.ConnectionStateChanged.RegisterEvent(onStateChanged, ref unregisterEvents);
            PrinterCommunication.Instance.WroteLine.RegisterEvent(Instance_WroteLine, ref unregisterEvents);
            PrinterCommunication.Instance.ActivePrintItemChanged.RegisterEvent(onActivePrintItemChanged, ref unregisterEvents);
        }

        public override void OnClosed(EventArgs e)
        {
            if (unregisterEvents != null)
            {
                unregisterEvents(this, null);
            }
            base.OnClosed(e);
        }

        public override void OnDraw(Graphics2D graphics2D)
        {
            timeSinceLastDrawTime.Restart();
            base.OnDraw(graphics2D);
        }

        void OnIdle(object state)
        {
            if (PrinterCommunication.Instance.PrinterIsPrinting)
            {
                if (!timeSinceLastDrawTime.IsRunning)
                {
                    timeSinceLastDrawTime.Start();
                }
                else if (timeSinceLastDrawTime.ElapsedMilliseconds > 999)
                {
                    UpdatePrintStatus();
                    timeSinceLastDrawTime.Restart();
                }
            }

            UiThread.RunOnIdle(OnIdle);
        }

        void Instance_WroteLine(object sender, EventArgs e)
        {
            UpdatePrintStatus();
        }

        private void onStateChanged(object sender, EventArgs e)
        {
            UpdatePrintStatus();
        }

        private void UpdatePrintStatus()
        {
            if (PrinterCommunication.Instance.ActivePrintItem != null)
            {
                
                //GC.WaitForFullGCComplete();

                string printPercentRemainingText;
                printPercentRemainingText = string.Format("{0:0.0}% complete", PrinterCommunication.Instance.PercentComplete);

                switch (PrinterCommunication.Instance.CommunicationState)
                {
                    case PrinterCommunication.CommunicationStates.PreparingToPrint:
                        activePrintLabel.Text = "Preparing To Print:";
                        //ActivePrintStatusText = ""; // set by slicer
                        activePrintInfo.Text = "";
                        break;

                    case PrinterCommunication.CommunicationStates.Printing:
                        {
                            activePrintLabel.Text = PrinterCommunication.Instance.PrintingStateString;
                            ActivePrintStatusText = printPercentRemainingText;
                        }
                        break;

                    case PrinterCommunication.CommunicationStates.Paused:
                        {
                            activePrintLabel.Text = "Printing Paused:";
                            ActivePrintStatusText = printPercentRemainingText;
                        }
                        break;

                    case PrinterCommunication.CommunicationStates.FinishedPrint:
                        activePrintLabel.Text = "Done Printing:";
                        ActivePrintStatusText = printPercentRemainingText;
                        break;

				default:
						string nextPrintLblActive = new LocalizedString ("Next Print").Translated;
						string nextPrintLblActiveFull = string.Format("{0}: ", nextPrintLblActive);

						activePrintLabel.Text = nextPrintLblActiveFull;
                        ActivePrintStatusText = "";
                        activePrintInfo.Text = "";
                        break;
                }
            }
            else
            {
				string nextPrintLabel = new LocalizedString ("Next Print").Translated;
				string nextPrintLabelFull = string.Format ("{0}:", nextPrintLabel);

				activePrintLabel.Text = nextPrintLabelFull;
				ActivePrintStatusText = string.Format(new LocalizedString("Press 'Add' to choose an item to print").Translated);
                activePrintInfo.Text = "";
            }
        }

        protected void onPrintItemChanged(object sender, EventArgs e)
        {
            UpdatePrintItemName();
            UpdatePrintStatus();
        }

        void UpdatePrintItemName()
        {
            if (PrinterCommunication.Instance.ActivePrintItem != null)
            {
                string labelName = textInfo.ToTitleCase(PrinterCommunication.Instance.ActivePrintItem.Name);
                labelName = labelName.Replace('_', ' ');
                this.activePrintName.Text = labelName;
            }
            else
            {
				this.activePrintName.Text = new LocalizedString("No items in the print queue").Translated;
            }
        }

        private TextWidget getPrintStatusLabel(string text, int pointSize)
        {
            TextWidget widget = new TextWidget(text, pointSize: pointSize);
            widget.TextColor = RGBA_Bytes.White;
            widget.AutoExpandBoundsToText = true;
            widget.MinimumSize = new Vector2(widget.Width, widget.Height);
            return widget;
        }
    }

    class ExtruderTemperatureWidget : IndicatorWidget
    {
        public ExtruderTemperatureWidget()
            : base("150.3°")
        {
            AddHandlers();
            setToCurrentTemperature();


        }

        event EventHandler unregisterEvents;
        void AddHandlers()
        {
            PrinterCommunication.Instance.ExtruderTemperatureRead.RegisterEvent(onTemperatureRead, ref unregisterEvents);
            this.MouseEnterBounds += onMouseEnterBounds;
            this.MouseLeaveBounds += onMouseLeaveBounds;
        }

        public override void OnClosed(EventArgs e)
        {
            if (unregisterEvents != null)
            {
                unregisterEvents(this, null);
            }
            base.OnClosed(e);
        }

        void onMouseEnterBounds(Object sender, EventArgs e)
        {
			HelpTextWidget.Instance.ShowHoverText(new LocalizedString("Extruder Temperature").Translated);
        }

        void onMouseLeaveBounds(Object sender, EventArgs e)
        {
            HelpTextWidget.Instance.HideHoverText();
        }

        void setToCurrentTemperature()
        {
            string tempDirectionIndicator = "";
            if (PrinterCommunication.Instance.TargetExtruderTemperature > 0)
            {
                if ((int)(PrinterCommunication.Instance.TargetExtruderTemperature + 0.5) < (int)(PrinterCommunication.Instance.ActualExtruderTemperature + 0.5))
                {
                    tempDirectionIndicator = "↓";
                }
                else if ((int)(PrinterCommunication.Instance.TargetExtruderTemperature + 0.5) > (int)(PrinterCommunication.Instance.ActualExtruderTemperature + 0.5))
                {
                    tempDirectionIndicator = "↑";
                }
            }
            this.IndicatorValue = string.Format("{0:0.#}°{1}", PrinterCommunication.Instance.ActualExtruderTemperature, tempDirectionIndicator);
        }

        void onTemperatureRead(Object sender, EventArgs e)
        {
            setToCurrentTemperature();
        }
    }

    class BedTemperatureWidget : IndicatorWidget
    {
        //Not currently hooked up to anything
        public BedTemperatureWidget()
            : base("150.3°")
        {
            AddHandlers();
            setToCurrentTemperature();
        }

        event EventHandler unregisterEvents;
        void AddHandlers()
        {
            PrinterCommunication.Instance.BedTemperatureRead.RegisterEvent(onTemperatureRead, ref unregisterEvents);
            this.MouseEnterBounds += onMouseEnterBounds;
            this.MouseLeaveBounds += onMouseLeaveBounds;
        }

        public override void OnClosed(EventArgs e)
        {
            if (unregisterEvents != null)
            {
                unregisterEvents(this, null);
            }
            base.OnClosed(e);
        }

        void onMouseEnterBounds(Object sender, EventArgs e)
        {
			HelpTextWidget.Instance.ShowHoverText(new LocalizedString("Bed Temperature").Translated);
        }

        void onMouseLeaveBounds(Object sender, EventArgs e)
        {
            HelpTextWidget.Instance.HideHoverText();

        }

        void setToCurrentTemperature()
        {
            this.IndicatorValue = string.Format("{0:0.#}°", PrinterCommunication.Instance.ActualBedTemperature);
        }

        void onTemperatureRead(Object sender, EventArgs e)
        {
            setToCurrentTemperature();
        }
    }

    class IndicatorWidget : GuiWidget
    {
        TextWidget indicatorTextWidget;
        RGBA_Bytes borderColor = new RGBA_Bytes(255, 255, 255);
        int borderWidth = 2;

        public string IndicatorValue
        {
            get
            {
                return indicatorTextWidget.Text;
            }
            set
            {
                if (indicatorTextWidget.Text != value)
                {
                    indicatorTextWidget.Text = value;
                }
            }
        }

        event EventHandler unregisterEvents;
        public IndicatorWidget(string textValue)
            : base(52, 52)
        {
            this.BackgroundColor = new RGBA_Bytes(255, 255, 255, 200);
            indicatorTextWidget = new TextWidget(textValue, pointSize: 11);
            indicatorTextWidget.TextColor = ActiveTheme.Instance.PrimaryAccentColor;
            indicatorTextWidget.HAnchor = HAnchor.ParentCenter;
            indicatorTextWidget.VAnchor = VAnchor.ParentCenter;
            indicatorTextWidget.AutoExpandBoundsToText = true;
            this.Margin = new BorderDouble(0, 2);
            this.AddChild(indicatorTextWidget);
            ActiveTheme.Instance.ThemeChanged.RegisterEvent(onThemeChanged, ref unregisterEvents);
        }

        public override void OnClosed(EventArgs e)
        {
            if (unregisterEvents != null)
            {
                unregisterEvents(this, null);
            }
            base.OnClosed(e);
        }

        private void onThemeChanged(object sender, EventArgs e)
        {
            this.indicatorTextWidget.TextColor = ActiveTheme.Instance.PrimaryAccentColor;
            this.Invalidate();
        }

        public override void OnDraw(Graphics2D graphics2D)
        {
            base.OnDraw(graphics2D);

            RectangleDouble Bounds = LocalBounds;
            RoundedRect borderRect = new RoundedRect(this.LocalBounds, 0);
            Stroke strokeRect = new Stroke(borderRect, borderWidth);
            graphics2D.Render(strokeRect, borderColor);
        }
    }
}
