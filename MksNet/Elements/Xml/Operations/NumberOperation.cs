using System;
using System.Collections.Generic;
using System.Text;

namespace MksNet.Elements.Xml.Operations
{
    internal class NumberOperation : IOperation
    {
        private double value;

        internal NumberOperation(double value)
        {
            this.value = value;
        }

        public double Resolve(ElementParameter parameter) => value;
    }
}
