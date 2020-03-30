using MksNet.Elements.Xml.Operations.Matrix;
using MksNet.Elements.Xml.Operations.Vector;
using System;
using System.Collections.Generic;
using System.Text;

namespace MksNet.Elements
{
    internal class ElementFactoryEntryFrame
    {
        internal string Name;

        internal string Reference;

        internal IVectorOperation TranslationOperation;

        internal IMatrixOperation RotationOperation;

    }
}
