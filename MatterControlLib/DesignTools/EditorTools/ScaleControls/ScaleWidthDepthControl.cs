﻿/*
Copyright (c) 2014, Lars Brubaker
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

using MatterHackers.Agg;
using MatterHackers.Agg.UI;
using MatterHackers.DataConverters3D;
using MatterHackers.MatterControl;
using MatterHackers.MatterControl.CustomWidgets;
using MatterHackers.MatterControl.DesignTools;
using MatterHackers.MatterControl.DesignTools.Operations;
using MatterHackers.MatterControl.PartPreviewWindow;
using MatterHackers.MeshVisualizer;
using MatterHackers.PolygonMesh;
using MatterHackers.RayTracer;
using MatterHackers.RenderOpenGl;
using MatterHackers.VectorMath;
using System;
using System.Collections.Generic;

namespace MatterHackers.Plugins.EditorTools
{
	public class ScaleWidthDepthControl : Object3DControl
	{
		/// <summary>
		/// Edge starting from the back (+y) going ccw
		/// </summary>
		private readonly int edgeIndex;

		private readonly Mesh minXminYMesh;

		private readonly double selectCubeSize = 7 * GuiWidget.DeviceScale;

		private readonly ThemeConfig theme;

		private readonly InlineEditControl xValueDisplayInfo;

		private readonly InlineEditControl yValueDisplayInfo;

		private bool hadClickOnControl;

		private PlaneShape hitPlane;

		private Vector3 initialHitPosition;

		private Vector3 originalPointToMove;

		private Vector2 sizeOnMouseDown;

		public ScaleWidthDepthControl(IObject3DControlContext context, int edgeIndex)
			: base(context)
		{
			theme = MatterControl.AppContext.Theme;

			xValueDisplayInfo = new InlineEditControl()
			{
				ForceHide = ForceHideScale,
				GetDisplayString = (value) => "{0:0.0}".FormatWith(value),
			};

			xValueDisplayInfo.EditComplete += EditComplete;

			xValueDisplayInfo.VisibleChanged += (s, e) =>
			{
				if (!xValueDisplayInfo.Visible)
				{
					hadClickOnControl = false;
				}
			};

			yValueDisplayInfo = new InlineEditControl()
			{
				ForceHide = ForceHideScale,
				GetDisplayString = (value) => "{0:0.0}".FormatWith(value)
			};

			yValueDisplayInfo.EditComplete += EditComplete;

			yValueDisplayInfo.VisibleChanged += (s, e) =>
			{
				if (!yValueDisplayInfo.Visible)
				{
					hadClickOnControl = false;
				}
			};

			if (edgeIndex % 2 == 1)
			{
				Object3DControlContext.GuiSurface.AddChild(xValueDisplayInfo);
			}
			else
			{
				Object3DControlContext.GuiSurface.AddChild(yValueDisplayInfo);
			}

			this.edgeIndex = edgeIndex;

			DrawOnTop = true;

			minXminYMesh = PlatonicSolids.CreateCube(selectCubeSize, selectCubeSize, selectCubeSize);

			CollisionVolume = minXminYMesh.CreateBVHData();

			Object3DControlContext.GuiSurface.BeforeDraw += Object3DControl_BeforeDraw;
		}

		public IObject3D ActiveSelectedItem { get; set; }

		private double DistToStart => 10 * GuiWidget.DeviceScale;

		private double LineLength => 35 * GuiWidget.DeviceScale;

		public static Vector3 GetScalingConsideringShiftKey(AxisAlignedBoundingBox originalSelectedBounds,
			AxisAlignedBoundingBox mouseDownSelectedBounds,
			Vector3 newSize,
			Keys modifierKeys)
		{
			var minimumSize = .1;
			var scaleAmount = Vector3.One;

			if (originalSelectedBounds.XSize <= 0
				|| originalSelectedBounds.YSize <= 0
				|| originalSelectedBounds.ZSize <= 0)
			{
				// don't scale if any dimension will go to 0
				return scaleAmount;
			}

			if (modifierKeys == Keys.Shift)
			{
				newSize.X = newSize.X <= minimumSize ? minimumSize : newSize.X;
				newSize.Y = newSize.Y <= minimumSize ? minimumSize : newSize.Y;
				newSize.Z = newSize.Z <= minimumSize ? minimumSize : newSize.Z;

				scaleAmount.X = mouseDownSelectedBounds.XSize / originalSelectedBounds.XSize;
				scaleAmount.Y = mouseDownSelectedBounds.YSize / originalSelectedBounds.YSize;
				scaleAmount.Z = mouseDownSelectedBounds.ZSize / originalSelectedBounds.ZSize;

				double scaleFromOriginal = Math.Max(newSize.X / mouseDownSelectedBounds.XSize, newSize.Y / mouseDownSelectedBounds.YSize);
				scaleFromOriginal = Math.Max(scaleFromOriginal, newSize.Z / mouseDownSelectedBounds.ZSize);
				scaleAmount *= scaleFromOriginal;
			}
			else
			{
				if (newSize.X > 0)
				{
					newSize.X = newSize.X <= minimumSize ? minimumSize : newSize.X;
					scaleAmount.X = newSize.X / originalSelectedBounds.XSize;
				}

				if (newSize.Y > 0)
				{
					newSize.Y = newSize.Y <= minimumSize ? minimumSize : newSize.Y;
					scaleAmount.Y = newSize.Y / originalSelectedBounds.YSize;
				}

				if (newSize.Z > 0)
				{
					newSize.Z = newSize.Z <= minimumSize ? minimumSize : newSize.Z;
					scaleAmount.Z = newSize.Z / originalSelectedBounds.ZSize;
				}
			}

			return scaleAmount;
		}

		public override void CancelOperation()
		{
			IObject3D selectedItem = RootSelection;
			if (selectedItem != null
				&& MouseDownOnControl)
			{
				if (selectedItem is IObjectWithWidthAndDepth widthDepthItem)
				{
					widthDepthItem.Width = sizeOnMouseDown.X;
					widthDepthItem.Depth = sizeOnMouseDown.Y;
				}

				MouseDownOnControl = false;
				MouseIsOver = false;

				Object3DControlContext.Scene.DrawSelection = true;
				Object3DControlContext.Scene.ShowSelectionShadow = true;
			}

			base.CancelOperation();
		}

		public override void Dispose()
		{
			yValueDisplayInfo.Close();
			Object3DControlContext.GuiSurface.BeforeDraw -= Object3DControl_BeforeDraw;
		}

		public override void Draw(DrawGlContentEventArgs e)
		{
			bool shouldDrawScaleControls = true;
			if (Object3DControlContext.SelectedObject3DControl != null
				&& Object3DControlContext.SelectedObject3DControl as ScaleMatrixEdgeControl == null)
			{
				shouldDrawScaleControls = false;
			}

			var selectedItem = RootSelection;

			if (selectedItem != null)
			{
				// Ensures that functions in this scope run against the original instance reference rather than the
				// current value, thus avoiding null reference errors that would occur otherwise

				if (shouldDrawScaleControls)
				{
					// don't draw if any other control is dragging
					if (MouseIsOver || MouseDownOnControl)
					{
						GLHelper.Render(minXminYMesh, theme.PrimaryAccentColor.WithAlpha(e.Alpha0to255), TotalTransform, RenderTypes.Shaded);
					}
					else
					{
						GLHelper.Render(minXminYMesh, theme.TextColor.Blend(theme.BackgroundColor, .35).WithAlpha(e.Alpha0to255), TotalTransform, RenderTypes.Shaded);
					}
				}

				if (hitPlane != null)
				{
					//Object3DControlContext.World.RenderPlane(hitPlane.Plane, Color.Red, true, 50, 3);
					//Object3DControlContext.World.RenderPlane(initialHitPosition, hitPlane.Plane.Normal, Color.Red, true, 50, 3);
				}
			}

			if (MouseIsOver || MouseDownOnControl)
			{
				DrawMeasureLines(e);
			}

			base.Draw(e);
		}

		public Vector3 GetCornerPosition(IObject3D item, int quadrantIndex)
		{
			quadrantIndex = quadrantIndex % 4;
			AxisAlignedBoundingBox originalSelectedBounds = item.GetAxisAlignedBoundingBox(item.Matrix.Inverted);
			Vector3 cornerPosition = originalSelectedBounds.GetBottomCorner(quadrantIndex);

			return cornerPosition.Transform(item.Matrix);
		}

		public Vector3 GetEdgePosition(IObject3D item, int edegIndex)
		{
			edegIndex = edegIndex % 4;
			AxisAlignedBoundingBox aabb = item.GetAxisAlignedBoundingBox(item.Matrix.Inverted);
			var edgePosition = default(Vector3);
			switch (edegIndex)
			{
				case 0:
					edgePosition = new Vector3(aabb.Center.X, aabb.MaxXYZ.Y, aabb.MinXYZ.Z);
					break;

				case 1:
					edgePosition = new Vector3(aabb.MinXYZ.X, aabb.Center.Y, aabb.MinXYZ.Z);
					break;

				case 2:
					edgePosition = new Vector3(aabb.Center.X, aabb.MinXYZ.Y, aabb.MinXYZ.Z);
					break;

				case 3:
					edgePosition = new Vector3(aabb.MaxXYZ.X, aabb.Center.Y, aabb.MinXYZ.Z);
					break;
			}

			return edgePosition.Transform(item.Matrix);
		}

		public override void OnMouseDown(Mouse3DEventArgs mouseEvent3D)
		{
			var selectedItem = RootSelection;
			ActiveSelectedItem = selectedItem;

			if (mouseEvent3D.MouseEvent2D.Button == MouseButtons.Left
				&& mouseEvent3D.info != null
				&& selectedItem != null)
			{
				hadClickOnControl = true;

				xValueDisplayInfo.Visible = true;
				yValueDisplayInfo.Visible = true;

				var edge = GetEdgePosition(selectedItem, edgeIndex);
				var otherSide = GetEdgePosition(selectedItem, edgeIndex + 2);
				originalPointToMove = edge;

				var upNormal = (edge - otherSide).GetNormal();
				var sideNormal = upNormal.Cross(mouseEvent3D.MouseRay.directionNormal).GetNormal();
				var planeNormal = upNormal.Cross(sideNormal).GetNormal();
				hitPlane = new PlaneShape(new Plane(planeNormal, mouseEvent3D.info.HitPosition), null);

				initialHitPosition = mouseEvent3D.info.HitPosition;
				if (selectedItem is IObjectWithWidthAndDepth widthDepthItem)
				{
					sizeOnMouseDown = new Vector2(widthDepthItem.Width, widthDepthItem.Depth);
				}
			}

			base.OnMouseDown(mouseEvent3D);
		}

		public override async void OnMouseMove(Mouse3DEventArgs mouseEvent3D, bool mouseIsOver)
		{
			var selectedItem = RootSelection;
			ActiveSelectedItem = selectedItem;

			if (MouseIsOver || MouseDownOnControl)
			{
				xValueDisplayInfo.Visible = true;
				yValueDisplayInfo.Visible = true;
			}
			else if (!hadClickOnControl
				|| (selectedItem is IObjectWithWidthAndDepth widthDepthItem 
					&& (widthDepthItem.Width != sizeOnMouseDown.X || widthDepthItem.Depth != sizeOnMouseDown.Y)))
			{
				xValueDisplayInfo.Visible = false;
				yValueDisplayInfo.Visible = false;
			}

			if (MouseDownOnControl && hitPlane != null)
			{
				var info = hitPlane.GetClosestIntersection(mouseEvent3D.MouseRay);

				if (info != null
					&& selectedItem != null)
				{
					var delta = info.HitPosition - initialHitPosition;

					var lockedEdge = GetEdgePosition(selectedItem, edgeIndex + 2);

					var stretchDirection = (GetEdgePosition(selectedItem, edgeIndex) - lockedEdge).GetNormal();
					var deltaAlongStretch = stretchDirection.Dot(delta);

					// scale it
					if (selectedItem is IObjectWithWidthAndDepth widthDepthItem)
					{
						var newSize = new Vector3(widthDepthItem.Width, widthDepthItem.Depth, 0);
						if (edgeIndex % 2 == 1)
						{
							newSize.X = sizeOnMouseDown.X + deltaAlongStretch;
							newSize.X = Math.Max(Math.Max(newSize.X, .001), Object3DControlContext.SnapGridDistance);
						}
						else
						{
							newSize.Y = sizeOnMouseDown.Y + deltaAlongStretch;
							newSize.Y = Math.Max(Math.Max(newSize.Y, .001), Object3DControlContext.SnapGridDistance);
						}

						if (Object3DControlContext.SnapGridDistance > 0)
						{
							// snap this position to the grid
							double snapGridDistance = Object3DControlContext.SnapGridDistance;

							// snap this position to the grid
							if (edgeIndex % 2 == 1)
							{
								newSize.X = ((int)((newSize.X / snapGridDistance) + .5)) * snapGridDistance;
							}
							else
							{
								newSize.Y = ((int)((newSize.Y / snapGridDistance) + .5)) * snapGridDistance;
							}
						}

						widthDepthItem.Width = newSize.X;
						widthDepthItem.Depth = newSize.Y;
						selectedItem.Invalidate(new InvalidateArgs(selectedItem, InvalidateType.DisplayValues));
					}

					await selectedItem.Rebuild();

					// and keep the locked edge in place
					Vector3 newLockedEdge = GetEdgePosition(selectedItem, edgeIndex + 2);

					selectedItem.Matrix *= Matrix4X4.CreateTranslation(lockedEdge - newLockedEdge);

					Invalidate();
				}
			}

			base.OnMouseMove(mouseEvent3D, mouseIsOver);
		}

		private void SetWidthDepthUndo(Vector2 doWidthDepth, Vector2 undoWidthDepth)
		{
			var activeSelectedItem = RootSelection;
			if (activeSelectedItem is IObjectWithWidthAndDepth widthDepthItem)
			{
				var undoBuffer = Object3DControlContext.Scene.UndoBuffer;
				undoBuffer.AddAndDo(new UndoRedoActions(() =>
				{
					widthDepthItem.Width = undoWidthDepth.X;
					widthDepthItem.Depth = undoWidthDepth.Y;
					activeSelectedItem?.Invalidate(new InvalidateArgs(activeSelectedItem, InvalidateType.Properties));
				},
				() =>
				{
					widthDepthItem.Width = doWidthDepth.X;
					widthDepthItem.Depth = doWidthDepth.Y;
					activeSelectedItem?.Invalidate(new InvalidateArgs(activeSelectedItem, InvalidateType.Properties));
				}));
			}
		}

		public override void OnMouseUp(Mouse3DEventArgs mouseEvent3D)
		{
			if (hadClickOnControl
				&& RootSelection is IObjectWithWidthAndDepth widthDepthItem
				&& (widthDepthItem.Width != sizeOnMouseDown.X || widthDepthItem.Depth != sizeOnMouseDown.Y))
			{
				SetWidthDepthUndo(new Vector2(widthDepthItem.Width, widthDepthItem.Depth), sizeOnMouseDown);
			}

			base.OnMouseUp(mouseEvent3D);
		}

		private void GetMeasureLine(out Vector3 start, out Vector3 end)
		{
			var selectedItem = RootSelection;
			var corner = new Vector3[4];
			var screen = new Vector3[4];
			for (int i = 0; i < 4; i++)
			{
				corner[i] = GetCornerPosition(selectedItem, edgeIndex + i);
				screen[i] = Object3DControlContext.World.GetScreenSpace(corner[i]);
			}

			// do we lie the start or the end of our edge
			if (screen[0].Z < screen[1].Z)
			{
				start = corner[0];
				end = corner[3];
			}
			else
			{
				start = corner[1];
				end = corner[2];
			}
		}

		private void DrawMeasureLines(DrawGlContentEventArgs e)
		{
			GetMeasureLine(out Vector3 start, out Vector3 end);

			var color = theme.TextColor;
			if (!e.ZBuffered)
			{
				theme.TextColor.WithAlpha(Constants.LineAlpha);
			}

			Frustum clippingFrustum = Object3DControlContext.World.GetClippingFrustum();

			Object3DControlContext.World.Render3DLine(clippingFrustum, start, end, color, e.ZBuffered, GuiWidget.DeviceScale);
		}

		public override void SetPosition(IObject3D selectedItem, MeshSelectInfo selectInfo)
		{
			// create the transform for the box
			Vector3 edgePosition = GetEdgePosition(selectedItem, edgeIndex);

			Vector3 boxCenter = edgePosition;

			double distBetweenPixelsWorldSpace = Object3DControlContext.World.GetWorldUnitsPerScreenPixelAtPosition(edgePosition);
			switch (edgeIndex)
			{
				case 0:
					boxCenter.Y += selectCubeSize / 2 * distBetweenPixelsWorldSpace;
					break;

				case 1:
					boxCenter.X -= selectCubeSize / 2 * distBetweenPixelsWorldSpace;
					break;

				case 2:
					boxCenter.Y -= selectCubeSize / 2 * distBetweenPixelsWorldSpace;
					break;

				case 3:
					boxCenter.X += selectCubeSize / 2 * distBetweenPixelsWorldSpace;
					break;
			}

			boxCenter.Z += selectCubeSize / 2 * distBetweenPixelsWorldSpace;

			var centerMatrix = Matrix4X4.CreateTranslation(boxCenter);
			centerMatrix = Matrix4X4.CreateScale(distBetweenPixelsWorldSpace) * centerMatrix;
			TotalTransform = centerMatrix;
		}

		private void EditComplete(object s, EventArgs e)
		{
			var selectedItem = ActiveSelectedItem;

			Vector3 lockedEdge = GetEdgePosition(selectedItem, edgeIndex + 2);

			Vector3 newSize = Vector3.Zero;
			newSize.X = xValueDisplayInfo.Value;
			newSize.Y = yValueDisplayInfo.Value;

			if (selectedItem is IObjectWithWidthAndDepth widthDepthItem)
			{
				SetWidthDepthUndo(new Vector2(widthDepthItem.Width, widthDepthItem.Depth), sizeOnMouseDown);
			}

			// and keep the locked edge in place
			Vector3 newLockedEdge = GetEdgePosition(selectedItem, edgeIndex + 2);

			selectedItem.Matrix *= Matrix4X4.CreateTranslation(lockedEdge - newLockedEdge);

			Invalidate();
		}

		private bool ForceHideScale()
		{
			var selectedItem = RootSelection;
			// if the selection changes
			if (selectedItem != ActiveSelectedItem)
			{
				return true;
			}

			// if another control gets a hover
			if (Object3DControlContext.HoveredObject3DControl != this
			&& Object3DControlContext.HoveredObject3DControl != null)
			{
				return true;
			}

			// if we clicked on the control
			if (hadClickOnControl)
			{
				return false;
			}

			return false;
		}

		private Vector3 GetDeltaToOtherSideXy(IObject3D selectedItem, int quadrantIndex)
		{
			Vector3 cornerPosition = GetCornerPosition(selectedItem, quadrantIndex);
			Vector3 cornerPositionCcw = GetCornerPosition(selectedItem, quadrantIndex + 1);
			Vector3 cornerPositionCw = GetCornerPosition(selectedItem, quadrantIndex + 3);

			double xDirection = cornerPositionCcw.X - cornerPosition.X;
			if (xDirection == 0)
			{
				xDirection = cornerPositionCw.X - cornerPosition.X;
			}

			double yDirection = cornerPositionCcw.Y - cornerPosition.Y;
			if (yDirection == 0)
			{
				yDirection = cornerPositionCw.Y - cornerPosition.Y;
			}

			return new Vector3(xDirection, yDirection, cornerPosition.Z);
		}

		private void Object3DControl_BeforeDraw(object sender, DrawEventArgs drawEvent)
		{
			var selectedItem = RootSelection;

			if (selectedItem != null)
			{
				if (MouseIsOver || MouseDownOnControl)
				{
					GetMeasureLine(out Vector3 start, out Vector3 end);
					var screenStart = Object3DControlContext.World.GetScreenSpace(start);
					var screenEnd = Object3DControlContext.World.GetScreenSpace(end);

					if (edgeIndex % 2 == 1)
					{
						xValueDisplayInfo.Value = (start - end).Length;
						xValueDisplayInfo.OriginRelativeParent = new Vector2((screenStart - screenEnd) /2 );
					}
					else
					{
						yValueDisplayInfo.Value = (start - end).Length;
						yValueDisplayInfo.OriginRelativeParent = new Vector2((screenStart - screenEnd) / 2);
					}
				}
			}
		}
	}
}