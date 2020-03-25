using System;
using System.Collections.Generic;
using System.Text;

namespace MksNet.Elements.Xml.Operations
{
    internal interface IOperation
    {
        double Resolve(ElementParameter parameter);
    }
}
