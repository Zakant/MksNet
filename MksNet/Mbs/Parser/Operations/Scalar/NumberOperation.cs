using System;
using System.Collections.Generic;
using System.Text;

namespace MksNet.Mbs.Parser.Operations.Scalar
{
    internal class NumberOperation : IScalarOperation
    {
        private double value;

        internal NumberOperation(double value)
        {
            this.value = value;
        }

        public double Resolve(Parameter parameter) => value;
    }
}
