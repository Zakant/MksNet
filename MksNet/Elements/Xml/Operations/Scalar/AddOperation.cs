using System;
using System.Collections.Generic;
using System.Text;
using System.Linq;

namespace MksNet.Elements.Xml.Operations.Scalar
{
    internal class AddOperation : ListOperation
    {
        public AddOperation(List<IScalarOperation> operations) : base(operations) { }

        public override double Resolve(ElementParameter parameter) => operations.Select(x => x.Resolve(parameter)).Sum();
    }
}
