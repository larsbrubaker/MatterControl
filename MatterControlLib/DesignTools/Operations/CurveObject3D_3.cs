/*
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
using System.ComponentModel;
using System.ComponentModel.DataAnnotations;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using MatterHackers.Agg;
using MatterHackers.Agg.UI;
using MatterHackers.DataConverters3D;
using MatterHackers.Localizations;
using MatterHackers.MatterControl.DesignTools.Operations;
using MatterHackers.MatterControl.PartPreviewWindow;
using MatterHackers.MeshVisualizer;
using MatterHackers.PolygonMesh;
using MatterHackers.RenderOpenGl.OpenGl;
using MatterHackers.VectorMath;
using Newtonsoft.Json;

namespace MatterHackers.MatterControl.DesignTools
{
	public class CurveObject3D_3 : SourceContainerObject3D, IEditorDraw, IPropertyGridModifier
	{
		// this needs to serialize but not be editable (so public but not a get set)
		public Vector3 RotationOffset;

		public CurveObject3D_3()
		{
			Name = "Curve".Localize();
		}

		public enum Measure
		{
			Diameter,
			Angle,
			Turns
		}

		[SuppressUpdateOnChange]
		public Measure Specify { get; set; }

		[DisplayName("Bend Up")]
		public bool BendCcw { get; set; } = true;

		public double Diameter { get; set; } = double.MaxValue;

		[JsonIgnore]
		public double Angle
		{
			get
			{
				// Angle = Turns * 360
				return Turns * 360.0;
			}

			set
			{
				// Turns = Angle / 360;
				Turns = value / 360.0;
			}
		}


		[JsonIgnore]
		public double Turns
		{
			get
			{
				// Turns = XSize / (Tau * (Diameter / 2));
				var aabb = this.SourceItem.GetAxisAlignedBoundingBox();
				return aabb.XSize / (MathHelper.Tau * (Diameter / 2));
			}

			set
			{
				// Diameter = ((XSize / Turns) / Tau) * 2
				var aabb = this.SourceItem.GetAxisAlignedBoundingBox();
				Diameter = (aabb.XSize / value) / MathHelper.Tau * 2;
			}
		}

		[Range(3, 360, ErrorMessage = "Value for {0} must be between {1} and {2}.")]
		[Description("Ensures the rotated part has a minimum number of sides per complete rotation")]
		public double MinSidesPerRotation { get; set; } = 30;

		[Range(0, 100, ErrorMessage = "Value for {0} must be between {1} and {2}.")]
		[Description("Where to start the bend as a percent of the width of the part")]
		public double StartPercent { get; set; } = 50;

		public void DrawEditor(InteractionLayer layer, List<Object3DView> transparentMeshes, DrawEventArgs e, ref bool suppressNormalDraw)
		{
			if (layer.Scene.SelectedItem != null
				&& layer.Scene.SelectedItem.DescendantsAndSelf().Where((i) => i == this).Any())
			{
				// we want to measure the
				var currentMatrixInv = Matrix.Inverted;
				var aabb = this.GetAxisAlignedBoundingBox(currentMatrixInv);

				layer.World.RenderCylinderOutline(this.WorldMatrix(), Vector3.Zero, Diameter, aabb.ZSize, 150, Color.Red, Color.Transparent);

				layer.World.RenderCylinderOutline(this.WorldMatrix(),
					Vector3.Zero,
					Diameter,
					aabb.ZSize,
					(int)Math.Min(180, Math.Max(0, this.MinSidesPerRotation)),
					Color.Transparent,
					Color.Red);
			}

			// turn the lighting back on
			GL.Enable(EnableCap.Lighting);
		}

		public override Task Rebuild()
		{
			this.DebugDepth("Rebuild");

			bool valuesChanged = false;

			// ensure we have good values
			if (StartPercent < 0 || StartPercent > 100)
			{
				StartPercent = Math.Min(100, Math.Max(0, StartPercent));
				valuesChanged = true;
			}

			if (Diameter != double.MaxValue
				&& (Diameter < 1 || Diameter > 100000))
			{
				Diameter = Math.Min(100000, Math.Max(1, Diameter));
				valuesChanged = true;
			}

			if (MinSidesPerRotation < 0 || MinSidesPerRotation > 360)
			{
				MinSidesPerRotation = Math.Min(360, Math.Max(0, MinSidesPerRotation));
				valuesChanged = true;
			}

			var originalAabb = this.GetAxisAlignedBoundingBox();

			var rebuildLocks = this.RebuilLockAll();

			return ApplicationController.Instance.Tasks.Execute(
				"Curve".Localize(),
				null,
				(reporter, cancellationToken) =>
				{
					var startRotationOffset = RotationOffset;

					// remember the current matrix then clear it so the parts will rotate at the original wrapped position

					var sourceAabb = SourceItem.GetAxisAlignedBoundingBox();
					if (Diameter == double.MaxValue)
					{
						// uninitialized set to a reasonable value
						Diameter = (int)sourceAabb.XSize;
						// ensure that the editor display value is updated
						valuesChanged = true;
					}

					if (Diameter > 0)
					{
						var radius = Diameter / 2;
						var circumference = MathHelper.Tau * radius;
						var rotationCenter = new Vector3(sourceAabb.MinXYZ.X + (sourceAabb.MaxXYZ.X - sourceAabb.MinXYZ.X) * (StartPercent / 100), sourceAabb.MaxXYZ.Y + radius, sourceAabb.Center.Z);
						double numRotations = sourceAabb.XSize / circumference;
						double numberOfCuts = numRotations * MinSidesPerRotation;
						double cutSize = sourceAabb.XSize / numberOfCuts;
						double cutPosition = sourceAabb.MinXYZ.X + cutSize;
						var cuts = new List<double>();
						for (int i = 0; i < numberOfCuts; i++)
						{
							cuts.Add(cutPosition);
							cutPosition += cutSize;
						}

						RotationOffset = rotationCenter;
						if (!BendCcw)
						{
							// fix the stored center so we draw correctly
							RotationOffset.Y = sourceAabb.MinXYZ.Y - radius;
						}

						var curvedChildren = new List<IObject3D>();

						foreach (var sourceItem in SourceItem.VisibleMeshes())
						{
							var originalMesh = sourceItem.Mesh;
							var transformedMesh = originalMesh.Copy(CancellationToken.None);
							var itemMatrix = sourceItem.WorldMatrix(SourceItem);

							if (!BendCcw)
							{
								// rotate around so it will bend correctly
								itemMatrix *= Matrix4X4.CreateTranslation(0, -sourceAabb.MaxXYZ.Y, 0);
								itemMatrix *= Matrix4X4.CreateRotationX(MathHelper.Tau / 2);
								itemMatrix *= Matrix4X4.CreateTranslation(0, sourceAabb.MaxXYZ.Y - sourceAabb.YSize, 0);
							}

							// transform into this space
							transformedMesh.Transform(itemMatrix);

							// split the mesh along the x axis
							SplitMeshAlongX(transformedMesh, cuts, cutSize / 8);

							for (int i = 0; i < transformedMesh.Vertices.Count; i++)
							{
								var position = transformedMesh.Vertices[i];

								var angleToRotate = ((position.X - rotationCenter.X) / circumference) * MathHelper.Tau - MathHelper.Tau / 4;
								var distanceFromCenter = rotationCenter.Y - position.Y;

								var rotatePosition = new Vector3Float(Math.Cos(angleToRotate), Math.Sin(angleToRotate), 0) * distanceFromCenter;
								rotatePosition.Z = position.Z;
								transformedMesh.Vertices[i] = rotatePosition + new Vector3Float(rotationCenter.X, radius + sourceAabb.MaxXYZ.Y, 0);
							}

							// transform back into item local space
							transformedMesh.Transform(Matrix4X4.CreateTranslation(-RotationOffset) * itemMatrix.Inverted);

							transformedMesh.MarkAsChanged();
							transformedMesh.CalculateNormals();

							var cuvedChild = new Object3D()
							{
								Mesh = transformedMesh
							};
							cuvedChild.CopyWorldProperties(sourceItem, this, Object3DPropertyFlags.All);
							curvedChildren.Add(cuvedChild);
						}

						this.Children.Modify((list) =>
						{
							list.Clear();
							list.AddRange(curvedChildren);
						});

						// set the matrix back
						this.Translate(RotationOffset - startRotationOffset);
						rebuildLocks.Dispose();
					}

					if (valuesChanged)
					{
						Invalidate(InvalidateType.DisplayValues);
					}

					Parent?.Invalidate(new InvalidateArgs(this, InvalidateType.Children));

					return Task.CompletedTask;
				});
		}

		public static void SplitMeshAlongX(Mesh mesh, List<double> cuts, double onPlaneDistance)
		{
			for (int j = 0; j < cuts.Count; j++)
			{
				mesh.Split(new Plane(Vector3.UnitX, cuts[j]), onPlaneDistance, (clipData) =>
				{
					// if two distances are less than 0
					if ((clipData.Dist[0] < 0 && clipData.Dist[1] < 0)
						|| (clipData.Dist[1] < 0 && clipData.Dist[2] < 0)
						|| (clipData.Dist[2] < 0 && clipData.Dist[0] < 0))
					{
						return true;
					}

					return false;
				});
			}

			for (int j = cuts.Count - 1; j >= 0; j--)
			{
				mesh.Split(new Plane(Vector3.UnitX, cuts[j]), .1);
			}

			return;
		}

		public void UpdateControls(PublicPropertyChange change)
		{
			change.SetRowVisible(nameof(Diameter), () => Specify == Measure.Diameter);
			change.SetRowVisible(nameof(Angle), () => Specify == Measure.Angle);
			change.SetRowVisible(nameof(Turns), () => Specify == Measure.Turns);

			Invalidate(new InvalidateArgs(null, InvalidateType.DisplayValues));
		}
	}
}