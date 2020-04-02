using System;
using System.Collections.Generic;
using System.Text;

namespace MksNet.Mbs.Parser.Operations.Scalar
{
    internal class SinOperation : IScalarOperation
    {
        private IScalarOperation operation;

        internal SinOperation(IScalarOperation operation)
        {
            this.operation = operation;
        }

        public double Resolve(Parameter parameter) => Math.Sin(operation.Resolve(parameter));
    }
}
