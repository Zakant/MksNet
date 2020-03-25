using System;
using System.Collections.Generic;
using System.Text;

namespace MksNet.Elements.Xml.Operations
{
    internal interface IOperation<TResult>
    {
        TResult Resolve(ElementParameter parameter);
    }
}
