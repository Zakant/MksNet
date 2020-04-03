using MathNet.Numerics.LinearAlgebra;
using MksNet.Exceptions;
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
            if (parameter.HasMatrix(parameterName))
                return parameter.GetMatrix(parameterName);
            else
                throw new ParameterNotFoundException($"No matrix parameter named \"{parameterName}\" was found in the given parameter set.");
        }
    }
}
