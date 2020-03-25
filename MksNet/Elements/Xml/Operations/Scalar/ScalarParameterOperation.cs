using System;
using System.Collections.Generic;
using System.Text;

namespace MksNet.Elements.Xml.Operations.Scalar
{
    internal class ScalarParameterOperation : IScalarOperation
    {
        private string parameterName;

        internal ScalarParameterOperation(string parameterName)
        {
            this.parameterName = parameterName;
        }

        public double Resolve(ElementParameter parameter)
        {
            throw new NotImplementedException();
        }
    }
}
