﻿/*
Copyright (c) 2017, John Lewin
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
using System.Threading.Tasks;
using MatterHackers.Agg.Image;
using MatterHackers.Agg.UI;

namespace MatterHackers.MatterControl.Library
{
	public class DynamicContainerLink : ILibraryContainerLink, IThumbnail
	{
		private readonly Func<ILibraryContainer> containerCreator;

		private readonly Func<string> nameGetter;
		private readonly Action<string> nameSetter;

		private readonly ImageBuffer thumbnail;
		private readonly ImageBuffer microIcon;
		private readonly Func<bool> visibilityGetter;

		public DynamicContainerLink(Func<string> nameGetter,
			Action<string> nameSetter,
			ImageBuffer thumbnail,
			Func<ILibraryContainer> creator = null,
			Func<bool> visibilityResolver = null)
			: this(nameGetter, nameSetter, thumbnail, null, creator, visibilityResolver)
		{
		}

		public DynamicContainerLink(Func<string> nameGetter,
			Action<string> nameSetter,
			ImageBuffer thumbnail,
			ImageBuffer microIcon,
			Func<ILibraryContainer> creator = null,
			Func<bool> visibilityGetter = null)
		{
			this.thumbnail = thumbnail?.SetPreMultiply();
			this.microIcon = microIcon;
			if (microIcon != null)
			{
				thumbnail.NewGraphics2D().Render(microIcon,
					(thumbnail.Width - microIcon.Width) / 2,
					(thumbnail.Height - microIcon.Height) * .43);

				microIcon.SetPreMultiply();
			}

			this.nameGetter = nameGetter;
			this.nameSetter = nameSetter;
			this.containerCreator = creator;
			this.visibilityGetter = visibilityGetter ?? (() => true);
		}

		public string Category { get; set; }

		public DateTime DateCreated { get; } = DateTime.Now;

		public DateTime DateModified { get; } = DateTime.Now;

		public string ID { get; set; }

		public bool IsProtected { get; set; } = true;

		public bool IsReadOnly { get; set; } = false;

		public bool IsVisible => this.visibilityGetter();

		public string Name
		{
			get => nameGetter?.Invoke();
			set
			{
				nameSetter?.Invoke(value);
			}
		}

		public Task<ILibraryContainer> GetContainer(Action<double, string> reportProgress)
		{
			return Task.FromResult(this.containerCreator());
		}

		public Task<ImageBuffer> GetThumbnail(int width, int height)
		{
			if (microIcon != null
				&& width < 24 * GuiWidget.DeviceScale
				&& height < 24 * GuiWidget.DeviceScale)
			{
				return Task.FromResult(microIcon?.AlphaToPrimaryAccent());
			}

			return Task.FromResult(thumbnail?.AlphaToPrimaryAccent());
		}
	}
}