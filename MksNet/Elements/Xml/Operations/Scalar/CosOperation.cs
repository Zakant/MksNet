using System;
using System.Collections.Generic;
using System.Text;

namespace MksNet.Elements.Xml.Operations.Scalar
{
    internal class CosOperation : IScalarOperation
    {
        private IScalarOperation operation;

        internal CosOperation(IScalarOperation operation)
        {
            this.operation = operation;
        }

        public double Resolve(ElementParameter parameter) => Math.Cos(operation.Resolve(parameter));
    }
}
