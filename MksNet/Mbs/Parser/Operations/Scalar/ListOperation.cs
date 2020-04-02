using System;
using System.Collections.Generic;
using System.Text;

namespace MksNet.Mbs.Parser.Operations.Scalar
{
    internal abstract class ListOperation : IScalarOperation
    {
        protected List<IScalarOperation> operations;

        internal ListOperation(List<IScalarOperation> operations)
        {
            this.operations = operations;
        }

        public abstract double Resolve(Parameter parameter);
    }
}
