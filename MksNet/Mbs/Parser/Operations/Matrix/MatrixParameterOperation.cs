using MathNet.Numerics.LinearAlgebra;
using System;
using System.Collections.Generic;
using System.Text;

namespace MksNet.Mbs.Parser.Operations.Matrix
{
    internal class MatrixParameterOperation : IMatrixOperation
    {
        private string parameterName;

        internal MatrixParameterOperation(string parameterName)
        {
            this.parameterName = parameterName;
        }

        Matrix<double> IOperation<Matrix<double>>.Resolve(Parameter parameter)
        {
            throw new NotImplementedException();
        }
    }
}
