using MathNet.Numerics.LinearAlgebra;
using System;
using System.Collections.Generic;
using System.Text;

namespace MksNet.Elements.Xml.Operations.Vector
{
    internal class VectorParameterOperation : IVectorOperation
    {
        private string parameterName;

        internal VectorParameterOperation(string parameterName)
        {
            this.parameterName = parameterName;
        }

        Vector<double> IOperation<Vector<double>>.Resolve(ElementParameter parameter)
        {
            throw new NotImplementedException();
        }
    }
}
