using MathNet.Numerics.LinearAlgebra;
using System;
using System.Collections.Generic;
using System.Text;

namespace MksNet.Elements.Xml.Operations.Matrix
{
    internal class StaticMatrixOperation : IMatrixOperation
    {
        private Matrix<double> matrix;

        internal StaticMatrixOperation(Matrix<double> data)
        {
            matrix = data;
        }


        public Matrix<double> Resolve(ElementParameter parameter) => matrix;
    }
}
