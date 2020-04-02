using MksNet.Mbs.Parser.Operations.Matrix;
using MksNet.Mbs.Parser.Operations.Vector;
using System;
using System.Collections.Generic;
using System.Text;

namespace MksNet.Mbs.Elements.Internal
{
    internal class ElementFactoryEntryFrame
    {
        internal string Name;

        internal string Reference;

        internal IVectorOperation TranslationOperation;

        internal IMatrixOperation RotationOperation;

    }
}
