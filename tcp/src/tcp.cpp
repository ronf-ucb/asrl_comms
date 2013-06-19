#include <tcp/TcpNode.hpp>
#include <asrl/rosutil/node_utilities.hpp>

int main(int argc, char* argv[])
{
  return asrl::rosutil::spinNodeCatchException<asrl::TcpNode>( argc, argv, "tcp_node");
}
