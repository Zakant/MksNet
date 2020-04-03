using MathNet.Numerics.LinearAlgebra;
using System;
using System.Collections.Generic;
using System.Text;

namespace MksNet.Mbs.Parser.Operations.Matrix
{
    internal class IdentityMatrixOperation : IMatrixOperation
    {

        private int size;

        internal IdentityMatrixOperation(int size)
        {
            this.size = size;
        }

        public Matrix<double> Resolve(Parameter parameter) => CreateMatrix.DenseIdentity<double>(size);
    }
}
