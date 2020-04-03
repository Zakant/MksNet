using MksNet.Exceptions;
using System;
using System.Collections.Generic;
using System.Text;

namespace MksNet.Mbs.Parser.Operations.Scalar
{
    internal class ScalarParameterOperation : IScalarOperation
    {
        private string parameterName;

        internal ScalarParameterOperation(string parameterName)
        {
            this.parameterName = parameterName;
        }

        public double Resolve(Parameter parameter)
        {
            if (parameter.HasScalar(parameterName))
                return parameter.GetScalar(parameterName);
            else
                throw new ParameterNotFoundException($"No scalar parameter named \"{parameterName}\" was found in the given parameter set.");
        }
    }
}
