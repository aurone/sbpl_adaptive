#include <sbpl_adaptive/discrete_space_information/projection.h>

namespace adim {

Projection::~Projection()
{
}

Projection::Projection(
    const AdaptiveStateRepresentationPtr &src_rep,
    const AdaptiveStateRepresentationPtr &tgt_rep,
    bool executable)
:
    space_(nullptr),
    src_rep_(src_rep),
    tgt_rep_(tgt_rep),
    executable_(executable)
{
}

} // namespace adim

