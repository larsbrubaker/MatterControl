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
using MatterHackers.MatterControl.DataStorage;

namespace MatterHackers.MatterControl.Library
{
	public class PrintHistoryItem : ILibraryAssetStream
	{
		public PrintHistoryItem(PrintTask printTask)
		{
			this.PrintTask = printTask;
		}

		public PrintTask PrintTask { get; }

		public long FileSize => 0;

		public string ContentType => "";

		public string FileName => "";

		public string AssetPath => "";

		public string ID { get; } = Guid.NewGuid().ToString();

		public string Name
		{
			get => this.PrintTask.PrintName;
			set => this.PrintTask.PrintName = value;
		}

		public bool IsProtected => true;

		public bool IsVisible => true;

		public DateTime DateCreated => this.PrintTask.PrintStart;

		public DateTime DateModified => this.PrintTask.PrintEnd;

		public bool LocalContentExists => true;

		public string Category => "General";

		public Task<StreamAndLength> GetStream(Action<double, string> reportProgress)
		{
			throw new NotImplementedException();
		}
	}
}
