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

using System.Collections.Generic;
using System.Linq;
using MatterHackers.Agg.UI;
using MatterHackers.DataConverters3D;
using MatterHackers.DataConverters3D.UndoCommands;
using MatterHackers.Localizations;
using MatterHackers.MatterControl.PartPreviewWindow;
using Newtonsoft.Json;

namespace MatterHackers.MatterControl.DesignTools.Operations
{
	public class SourceContainerObject3D : Object3D
	{
		public override bool CanFlatten => true;

		private Object3D _sourceItem;
		public Object3D SourceItem
		{
			get => _sourceItem;

			set
			{
				// remove any existing listener
				if (_sourceItem != null)
				{
					_sourceItem.Invalidated -= SourceItem_Invalidated;
				}

				_sourceItem = value;

				// and listen for changes
				_sourceItem.Invalidated += SourceItem_Invalidated;
			}
		}

		private void SourceItem_Invalidated(object sender, InvalidateArgs e)
		{
			this.Invalidate(e);
		}

		public SourceContainerObject3D()
		{
			SourceItem = new Object3D();
		}

		public override void Flatten(UndoBuffer undoBuffer)
		{
			using (RebuildLock())
			{
				var newChildren = new List<IObject3D>();
				// push our matrix into a copy of our children
				foreach (var child in this.Children)
				{
					var newChild = child.Clone();
					newChildren.Add(newChild);
					newChild.Matrix *= this.Matrix;
					var flags = Object3DPropertyFlags.Visible;
					if (this.Color.alpha != 0)
					{
						flags |= Object3DPropertyFlags.Color;
					}

					if (this.OutputType != PrintOutputTypes.Default)
					{
						flags |= Object3DPropertyFlags.OutputType;
					}

					if (this.MaterialIndex != -1)
					{
						flags |= Object3DPropertyFlags.MaterialIndex;
					}

					newChild.CopyProperties(this, flags);
				}

				if (newChildren.Count > 1)
				{
					// wrap the children in an object so they remain a group
					var group = new Object3D();
					group.Children.Modify((groupList) =>
					{
						groupList.AddRange(newChildren);
					});

					newChildren.Clear();
					newChildren.Add(group);
				}

				// add flatten to the name to show what happened
				newChildren[0].Name = this.Name + " - " + "Flattened".Localize();

				// and replace us with the children
				var replaceCommand = new ReplaceCommand(new[] { this }, newChildren);
				if (undoBuffer != null)
				{
					undoBuffer.AddAndDo(replaceCommand);
				}
				else
				{
					replaceCommand.Do();
				}

				foreach (var child in newChildren[0].DescendantsAndSelf())
				{
					child.MakeNameNonColliding();
				}
			}

			Invalidate(InvalidateType.Children);
		}

		public override async void OnInvalidate(InvalidateArgs invalidateType)
		{
			// TODO: color and output type could have special consideration that would not require a rebuild
			// They could just propagate the color and output type to the correctly child and everything would be good
			if ((invalidateType.InvalidateType.HasFlag(InvalidateType.Children)
				|| invalidateType.InvalidateType.HasFlag(InvalidateType.Matrix)
				|| invalidateType.InvalidateType.HasFlag(InvalidateType.Mesh)
				|| invalidateType.InvalidateType.HasFlag(InvalidateType.Color)
				|| invalidateType.InvalidateType.HasFlag(InvalidateType.OutputType))
				&& invalidateType.Source != this
				&& !RebuildLocked)
			{
				await Rebuild();
			}
			else if (invalidateType.InvalidateType.HasFlag(InvalidateType.Properties)
				&& invalidateType.Source == this)
			{
				await Rebuild();
			}
			else
			{
				base.OnInvalidate(invalidateType);
			}
		}

		public async void WrapSelectedItemAndSelect(InteractiveScene scene)
		{
			using (RebuildLock())
			{
				var selectedItems = scene.GetSelectedItems();

				if (selectedItems.Count > 0)
				{
					// clear the selected item
					scene.SelectedItem = null;

					using (RebuildLock())
					{
						var clonedItemsToAdd = new List<IObject3D>(selectedItems.Select((i) => i.Clone()));

						Children.Modify((list) =>
						{
							list.Clear();
							list.AddRange(clonedItemsToAdd);
						});

						SourceItem.Children.Modify((list) =>
						{
							list.Clear();
							list.AddRange(clonedItemsToAdd);
						});
					}

					scene.UndoBuffer.AddAndDo(
						new ReplaceCommand(
							new List<IObject3D>(selectedItems),
							new List<IObject3D> { this }));

					await this.Rebuild();
				}
			}

			// and select this
			scene.SelectedItem = this;

			this.Invalidate(InvalidateType.Children);
		}


		public override void Remove(UndoBuffer undoBuffer)
		{
			using (RebuildLock())
			{
				// push our matrix into a copy of our children

				var newChild = SourceItem.Clone();
				newChild.Matrix *= this.Matrix;
				var flags = Object3DPropertyFlags.Visible;
				if (this.Color.alpha != 0)
				{
					flags |= Object3DPropertyFlags.Color;
				}

				if (this.OutputType != PrintOutputTypes.Default)
				{
					flags |= Object3DPropertyFlags.OutputType;
				}

				if (this.MaterialIndex != -1)
				{
					flags |= Object3DPropertyFlags.MaterialIndex;
				}

				newChild.CopyProperties(this, flags);

				// and replace us with the children
				var replaceCommand = new ReplaceCommand(new[] { this }, new[] { newChild }, false);
				if (undoBuffer != null)
				{
					undoBuffer.AddAndDo(replaceCommand);
				}
				else
				{
					replaceCommand.Do();
				}
			}

			Invalidate(InvalidateType.Children);
		}
	}
}