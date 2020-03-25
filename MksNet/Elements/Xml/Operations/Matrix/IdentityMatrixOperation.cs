using MathNet.Numerics.LinearAlgebra;
using System;
using System.Collections.Generic;
using System.Text;

namespace MksNet.Elements.Xml.Operations.Matrix
{
    internal class IdentityMatrixOperation : IMatrixOperation
    {
        public Matrix<double> Resolve(ElementParameter parameter) => CreateMatrix.DenseIdentity<double>(3);
    }
}
