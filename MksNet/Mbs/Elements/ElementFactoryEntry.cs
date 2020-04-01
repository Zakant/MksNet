﻿using MksNet.Mbs.Elements.Xml.Operations.Matrix;
using MksNet.Mbs.Elements.Xml.Operations.Scalar;
using System;
using System.Collections.Generic;
using System.Text;

namespace MksNet.Mbs.Elements
{
    internal class ElementFactoryEntry
    {

        internal string Name;

        internal string Author;

        internal string Description;

        internal string Url;


        internal IScalarOperation MassOperation;

        internal IMatrixOperation InertiaOperation;

        internal List<ElementFactoryEntryFrame> Frames;
    }
}