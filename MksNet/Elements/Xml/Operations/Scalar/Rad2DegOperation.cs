using System;
using System.Collections.Generic;
using System.Text;

namespace MksNet.Elements.Xml.Operations.Scalar
{
    internal class Rad2DegOperation : IScalarOperation
    {
        private IScalarOperation operation;

        internal Rad2DegOperation(IScalarOperation operation)
        {
            this.operation = operation;
        }

        public double Resolve(ElementParameter parameter) => 180.0 / Math.PI * operation.Resolve(parameter);
    }
}
