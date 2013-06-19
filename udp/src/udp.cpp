#include <udp/UdpNode.hpp>
#include <asrl/rosutil/node_utilities.hpp>

int main(int argc, char* argv[])
{
  return asrl::rosutil::spinNodeCatchException<asrl::UdpNode>( argc, argv, "udp_node");
}
