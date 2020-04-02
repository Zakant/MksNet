using System;
using System.Collections.Generic;
using System.Text;

namespace MksNet.Mbs.Parser.Operations
{
    internal interface IOperation<TResult>
    {
        TResult Resolve(Parameter parameter);
    }
}
