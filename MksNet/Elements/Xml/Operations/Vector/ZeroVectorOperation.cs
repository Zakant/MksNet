using MathNet.Numerics.LinearAlgebra;
using System;
using System.Collections.Generic;
using System.Text;

namespace MksNet.Elements.Xml.Operations.Vector
{
    internal class ZeroVectorOperation : IVectorOperation
    {
        public Vector<double> Resolve(ElementParameter parameter) => CreateVector.Dense<double>(3, 0);
    }
}
