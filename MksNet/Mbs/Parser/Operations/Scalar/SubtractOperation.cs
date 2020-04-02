using System;
using System.Collections.Generic;
using System.Text;
using System.Linq;

namespace MksNet.Mbs.Parser.Operations.Scalar
{
    internal class SubtractOperation : ListOperation
    {
        public SubtractOperation(List<IScalarOperation> operations) : base(operations) { }

        public override double Resolve(Parameter parameter)
        {
            var values = operations.Select(x => x.Resolve(parameter));
            double result = values.First();
            foreach (var x in values.Skip(1))
                result -= x;
            return result;
        }
    }
}
