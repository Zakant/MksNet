using MathNet.Numerics.LinearAlgebra;
using System;
using System.Collections.Generic;
using System.Text;

namespace MksNet.Mbs.Parser.Operations.Vector
{
    internal class ZeroVectorOperation : IVectorOperation
    {
        public Vector<double> Resolve(Parameter parameter) => CreateVector.Dense<double>(3, 0);
    }
}
