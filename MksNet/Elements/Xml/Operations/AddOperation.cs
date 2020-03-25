using System;
using System.Collections.Generic;
using System.Text;
using System.Linq;

namespace MksNet.Elements.Xml.Operations
{
    internal class AddOperation : ListOperation
    {
        public AddOperation(List<IOperation> operations) : base(operations) { }

        public override double Resolve(ElementParameter parameter) => operations.Select(x => x.Resolve(parameter)).Sum();
    }
}
