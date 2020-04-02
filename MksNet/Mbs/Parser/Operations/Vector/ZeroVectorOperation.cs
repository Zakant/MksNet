using MathNet.Numerics.LinearAlgebra;
using System;
using System.Collections.Generic;
using System.Text;

namespace MksNet.Mbs.Parser.Operations.Vector
{
    internal class ZeroVectorOperation : IVectorOperation
    {
        private int size;

        internal ZeroVectorOperation(int size)
        {
            this.size = size;
        }

        public Vector<double> Resolve(Parameter parameter) => CreateVector.Dense<double>(size, 0);
    }
}
