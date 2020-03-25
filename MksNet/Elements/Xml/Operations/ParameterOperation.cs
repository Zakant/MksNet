using System;
using System.Collections.Generic;
using System.Text;

namespace MksNet.Elements.Xml.Operations
{
    internal class ParameterOperation : IOperation
    {
        private string parameterName;

        internal ParameterOperation(string parameterName)
        {
            this.parameterName = parameterName;
        }

        public double Resolve(ElementParameter parameter)
        {
            throw new NotImplementedException();
        }
    }
}
