﻿/*
Copyright (c) 2019, John Lewin
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
using System.Threading.Tasks;
using MatterHackers.Agg;
using MatterHackers.Agg.Platform;
using MatterHackers.DataConverters3D;
using MatterHackers.Localizations;
using MatterHackers.MatterControl.DesignTools;
using MatterHackers.MatterControl.SlicerConfiguration;

namespace MatterHackers.MatterControl.Library
{
	public class PrimitivesContainer : LibraryContainer
	{
		public PrimitivesContainer()
		{
			Name = "Primitives".Localize();
			DefaultSort = new LibrarySortBehavior()
			{
				SortKey = SortKey.ModifiedDate,
				Ascending = true,
			};
		}

		public override void Load()
		{
			var library = ApplicationController.Instance.Library;

			long index = DateTime.Now.Ticks;
			var libraryItems = new List<GeneratorItem>()
			{
				new GeneratorItem(
					() => "Cube".Localize(),
					null,
					async () => await CubeObject3D.Create())
					{ DateCreated = new System.DateTime(index++) },
				new GeneratorItem(
					() => "Pyramid".Localize(),
					null,
					async () => await PyramidObject3D.Create())
					{ DateCreated = new System.DateTime(index++) },
				new GeneratorItem(
					() => "Wedge".Localize(),
					null,
					async () => await WedgeObject3D.Create())
					{ DateCreated = new System.DateTime(index++) },
				new GeneratorItem(
					() => "Half Wedge".Localize(),
					null,
					async () => await HalfWedgeObject3D.Create())
					{ DateCreated = new System.DateTime(index++) },
				new GeneratorItem(
					() => "Text".Localize(),
					null,
					async () => await TextObject3D.Create())
					{ DateCreated = new System.DateTime(index++) },
				new GeneratorItem(
					() => "Cylinder".Localize(),
					null,
					async () => await CylinderObject3D.Create())
					{ DateCreated = new System.DateTime(index++) },
				new GeneratorItem(
					() => "Cone".Localize(),
					null,
					async () => await ConeObject3D.Create())
					{ DateCreated = new System.DateTime(index++) },
				new GeneratorItem(
					() => "Half Cylinder".Localize(),
					null,
					async () => await HalfCylinderObject3D.Create())
					{ DateCreated = new System.DateTime(index++) },
				new GeneratorItem(
					() => "Torus".Localize(),
					null,
					async () => await TorusObject3D.Create())
					{ DateCreated = new System.DateTime(index++) },
				new GeneratorItem(
					() => "Ring".Localize(),
					null,
					async () => await RingObject3D.Create())
					{ DateCreated = new System.DateTime(index++) },
				new GeneratorItem(
					() => "Sphere".Localize(),
					null,
					async () => await SphereObject3D.Create())
					{ DateCreated = new System.DateTime(index++) },
				new GeneratorItem(
					() => "Half Sphere".Localize(),
					null,
					async () => await HalfSphereObject3D.Create())
					{ DateCreated = new System.DateTime(index++) },
#if DEBUG
				new GeneratorItem(
					() => "SCAD Script".Localize(),
					null,
					async () => await OpenScadScriptObject3D.Create())
					{ DateCreated = new System.DateTime(index++) },
#endif
				new GeneratorItem(
					() => "Image Converter".Localize(),
					null,
					() =>
					{
						// Construct an image
						var imageObject = new ImageObject3D()
						{
							AssetPath = StaticData.Instance.ToAssetPath(Path.Combine("Images", "mh-logo.png"))
						};

						// Construct a scene
						var bedConfig = new BedConfig(null);
						var tempScene = bedConfig.Scene;
						tempScene.Children.Add(imageObject);
						tempScene.SelectedItem = imageObject;

						// Invoke ImageConverter operation, passing image and scene
						SceneOperations.ById("ImageConverter").Action(bedConfig);

						// Return replacement object constructed in ImageConverter operation
						var constructedComponent = tempScene.SelectedItem;
						tempScene.Children.Remove(constructedComponent);

						return Task.FromResult<IObject3D>(constructedComponent);
					})
					{ DateCreated = new System.DateTime(index++) },
				new GeneratorItem(
					() => "Measure Tool".Localize(),
					null,
					async () => await MeasureToolObject3D.Create())
					{ DateCreated = new System.DateTime(index++) },
			};

			string title = "Primitive Shapes".Localize();

			foreach (var item in libraryItems)
			{
				item.Category = title;
				Items.Add(item);
			}
		}
	}
}
