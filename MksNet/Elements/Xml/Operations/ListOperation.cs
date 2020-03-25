using System;
using System.Collections.Generic;
using System.Text;

namespace MksNet.Elements.Xml.Operations
{
    internal abstract class ListOperation : IOperation
    {
        protected List<IOperation> operations;

        internal ListOperation(List<IOperation> operations)
        {
            this.operations = operations;
        }

        public abstract double Resolve(ElementParameter parameter);
    }
}
