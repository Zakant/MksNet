using MathNet.Numerics.LinearAlgebra;
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
            throw new NotImplementedException();
        }
    }
}
