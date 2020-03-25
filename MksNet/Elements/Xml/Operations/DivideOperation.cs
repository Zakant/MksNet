using System;
using System.Collections.Generic;
using System.Text;
using System.Linq;

namespace MksNet.Elements.Xml.Operations
{
    internal class DivideOperation : ListOperation
    {
        public DivideOperation(List<IOperation> operations) : base(operations) { }

        public override double Resolve(ElementParameter parameter)
        {
            var values = operations.Select(x => x.Resolve(parameter));
            double result = values.First();
            foreach (var x in values.Skip(1))
                result /= x;
            return result;
        }
    }
}
