using MathNet.Numerics.LinearAlgebra;
using System;
using System.Collections.Generic;
using System.Text;

namespace MksNet.Mbs.Parser.Operations.Matrix
{
    internal class IdentityMatrixOperation : IMatrixOperation
    {
        public Matrix<double> Resolve(Parameter parameter) => CreateMatrix.DenseIdentity<double>(3);
    }
}
