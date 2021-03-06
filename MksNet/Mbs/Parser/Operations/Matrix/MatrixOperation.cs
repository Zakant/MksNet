﻿using MathNet.Numerics.LinearAlgebra;
using MksNet.Mbs.Parser.Operations.Scalar;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace MksNet.Mbs.Parser.Operations.Matrix
{
    internal class MatrixOperation : IMatrixOperation
    {
        private IEnumerable<IEnumerable<IScalarOperation>> matrix;

        internal MatrixOperation(IEnumerable<IEnumerable<IScalarOperation>> data)
        {
            matrix = data;
        }

        public Matrix<double> Resolve(Parameter parameter) => CreateMatrix.DenseOfRows(matrix.Select(x => x.Select(y => y.Resolve(parameter))));
    }
}
