using System;
using System.Collections.Generic;
using System.Text;

namespace MksNet.Mbs.Elements.Xml.Operations.Scalar
{
    internal class SinOperation : IScalarOperation
    {
        private IScalarOperation operation;

        internal SinOperation(IScalarOperation operation)
        {
            this.operation = operation;
        }

        public double Resolve(ElementParameter parameter) => Math.Sin(operation.Resolve(parameter));
    }
}
