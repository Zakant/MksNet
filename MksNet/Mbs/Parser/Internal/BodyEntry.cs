using System;
using System.Collections.Generic;
using System.Text;

namespace MksNet.Mbs.Parser.Internal
{
    internal class BodyEntry
    {
        internal string Name { get; set; }

        internal string Type { get; set; }

        internal Parameter LocalParameters { get; set; }

        internal string LinkType { get; set; }

        internal string LocalFrame { get; set; }

        internal string RemoteBody { get; set; }

        internal string RemoteFrame { get; set; }
    }
}
