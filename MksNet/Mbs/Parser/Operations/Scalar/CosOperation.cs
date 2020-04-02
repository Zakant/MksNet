using System;
using System.Collections.Generic;
using System.Text;

namespace MksNet.Mbs.Parser.Operations.Scalar
{
    internal class CosOperation : IScalarOperation
    {
        private IScalarOperation operation;

        internal CosOperation(IScalarOperation operation)
        {
            this.operation = operation;
        }

        public double Resolve(Parameter parameter) => Math.Cos(operation.Resolve(parameter));
    }
}
