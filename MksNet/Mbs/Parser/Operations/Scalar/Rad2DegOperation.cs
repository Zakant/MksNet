using System;
using System.Collections.Generic;
using System.Text;

namespace MksNet.Mbs.Parser.Operations.Scalar
{
    internal class Rad2DegOperation : IScalarOperation
    {
        private IScalarOperation operation;

        internal Rad2DegOperation(IScalarOperation operation)
        {
            this.operation = operation;
        }

        public double Resolve(Parameter parameter) => 180.0 / Math.PI * operation.Resolve(parameter);
    }
}
