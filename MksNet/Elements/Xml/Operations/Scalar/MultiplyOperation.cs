using System;
using System.Collections.Generic;
using System.Text;
using System.Linq;

namespace MksNet.Elements.Xml.Operations.Scalar
{
    internal class MultiplyOperation : ListOperation
    {
        public MultiplyOperation(List<IScalarOperation> operations) : base(operations) { }

        public override double Resolve(ElementParameter parameter)
        {
            double result = 1;
            foreach (var x in operations.Select(x => x.Resolve(parameter)))
                result *= x;
            return result;
        }
    }
}
