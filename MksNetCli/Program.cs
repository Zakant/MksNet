using MksNet.Mbs;
using MksNet.Mbs.Elements;
using MksNet.Mbs.Joints;
using System;

namespace MksNetCli
{
    class Program
    {
        static void Main(string[] args)
        {
            Console.WriteLine("Hello World!");

            ElementFactory.Instance.LoadFolderDefinition(@".\Definitions\Elements");
            JointFactory.Instance.LoadFolderDefinition(@".\Definitions\Joints");
            MultibodySystem.LoadFromFile(@".\Definitions\Systems\SimplePendulum.xml");
        }
    }
}
