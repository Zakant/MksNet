using MathNet.Numerics.LinearAlgebra;
using MksNet.Exceptions;
using System;
using System.Collections.Generic;
using System.Text;

namespace MksNet.Mbs.Parser.Operations.Vector
{
    internal class VectorParameterOperation : IVectorOperation
    {
        private string parameterName;

        internal VectorParameterOperation(string parameterName)
        {
            this.parameterName = parameterName;
        }

        Vector<double> IOperation<Vector<double>>.Resolve(Parameter parameter)
        {
            if (parameter.HasVector(parameterName))
                return parameter.GetVector(parameterName);
            else
                throw new ParameterNotFoundException($"No vector parameter named \"{parameterName}\" was found in the given parameter set.");
        }
    }
}
