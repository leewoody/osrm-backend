#include "contractor/contractor.hpp"
#include "contractor/crc32_processor.hpp"
#include "contractor/files.hpp"
#include "contractor/graph_contractor.hpp"
#include "contractor/graph_contractor_adaptors.hpp"

#include "extractor/compressed_edge_container.hpp"
#include "extractor/edge_based_graph_factory.hpp"
#include "extractor/files.hpp"
#include "extractor/node_based_edge.hpp"

#include "storage/io.hpp"

#include "updater/updater.hpp"

#include "util/exception.hpp"
#include "util/exception_utils.hpp"
#include "util/graph_loader.hpp"
#include "util/integer_range.hpp"
#include "util/log.hpp"
#include "util/static_graph.hpp"
#include "util/string_util.hpp"
#include "util/timing_util.hpp"
#include "util/typedefs.hpp"

#include <algorithm>
#include <bitset>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <memory>
#include <vector>

namespace osrm
{
namespace contractor
{

std::vector<bool> ComputeCore(const ContractorConfig &config, std::size_t number_of_nodes)
{
    extractor::EdgeBasedNodeDataContainer node_data;
    extractor::files::readNodeData(config.GetPath(".osrm.ebg_nodes"), node_data);

    extractor::ProfileProperties properties;
    extractor::files::readProfileProperties(config.GetPath(".osrm.properties"), properties);

    std::vector<bool> core(number_of_nodes, false);
    for (const auto mask : properties.excludable_classes)
    {
        for (const auto node : util::irange<NodeID>(0, number_of_nodes))
        {
            core[node] = core[node] || (node_data.GetClassData(node) & mask) > 0;
        }
    }

    return core;
}

int Contractor::Run()
{
    if (config.core_factor > 1.0 || config.core_factor < 0)
    {
        throw util::exception("Core factor must be between 0.0 to 1.0 (inclusive)" + SOURCE_REF);
    }

    TIMER_START(preparing);

    util::Log() << "Reading node weights.";
    std::vector<EdgeWeight> node_weights;
    {
        storage::io::FileReader reader(config.GetPath(".osrm.enw"),
                                       storage::io::FileReader::VerifyFingerprint);
        storage::serialization::read(reader, node_weights);
    }
    util::Log() << "Done reading node weights.";

    util::Log() << "Loading edge-expanded graph representation";

    std::vector<extractor::EdgeBasedEdge> edge_based_edge_list;

    updater::Updater updater(config.updater_config);
    EdgeID max_edge_id = updater.LoadAndUpdateEdgeExpandedGraph(edge_based_edge_list, node_weights);

    // Contracting the edge-expanded graph

    TIMER_START(contraction);
    std::vector<bool> is_core_node;
    std::vector<float> node_levels;
    if (config.use_cached_priority)
    {
        files::readLevels(config.GetPath(".osrm.level"), node_levels);
    }

    util::DeallocatingVector<QueryEdge> contracted_edge_list;
    { // own scope to not keep the contractor around
        auto contractor_graph = toContractorGraph(max_edge_id + 1, std::move(edge_based_edge_list));

        auto core = ComputeCore(config, contractor_graph.GetNumberOfNodes());
        std::vector<bool> not_core(core.size());
        std::transform(core.begin(), core.end(), not_core.begin(), [](const bool is_core) {
            return !is_core;
        });

        // By not contracting all contractable nodes we avoid creating
        // a very dense core. This increases the overall graph sizes a little bit
        // but increases the final CH quality and contraction speed.
        constexpr float BASE_CORE = 0.9;
        std::vector<bool> overall_core;
        std::tie(std::ignore, overall_core) = contractGraph(
            contractor_graph, std::move(not_core), std::move(node_levels), node_weights, BASE_CORE);

        std::tie(node_levels, is_core_node) = contractGraph(contractor_graph,
                                                            std::move(overall_core),
                                                            std::move(node_levels),
                                                            std::move(node_weights),
                                                            config.core_factor);

        // Don't save the core for non-CoreCh
        if (config.core_factor == 1.0)
            is_core_node.clear();

        util::Log() << "Contracted graph has " << contractor_graph.GetNumberOfEdges() << " edges.";

        contracted_edge_list = toEdges<QueryEdge>(std::move(contractor_graph));
    }
    TIMER_STOP(contraction);

    util::Log() << "Contraction took " << TIMER_SEC(contraction) << " sec";

    {
        RangebasedCRC32 crc32_calculator;
        const unsigned checksum = crc32_calculator(contracted_edge_list);

        files::writeGraph(config.GetPath(".osrm.hsgr"),
                          checksum,
                          QueryGraph{max_edge_id + 1, std::move(contracted_edge_list)});
    }

    files::writeCoreMarker(config.GetPath(".osrm.core"), is_core_node);
    if (!config.use_cached_priority)
    {
        files::writeLevels(config.GetPath(".osrm.level"), node_levels);
    }

    TIMER_STOP(preparing);

    util::Log() << "Preprocessing : " << TIMER_SEC(preparing) << " seconds";

    util::Log() << "finished preprocessing";

    return 0;
}

} // namespace contractor
} // namespace osrm
