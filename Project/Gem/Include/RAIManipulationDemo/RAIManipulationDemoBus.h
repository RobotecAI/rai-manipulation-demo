
#pragma once

#include <RAIManipulationDemo/RAIManipulationDemoTypeIds.h>

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>

namespace RAIManipulationDemo
{
    class RAIManipulationDemoRequests
    {
    public:
        AZ_RTTI(RAIManipulationDemoRequests, RAIManipulationDemoRequestsTypeId);
        virtual ~RAIManipulationDemoRequests() = default;
        // Put your public methods here
    };

    class RAIManipulationDemoBusTraits
        : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using RAIManipulationDemoRequestBus = AZ::EBus<RAIManipulationDemoRequests, RAIManipulationDemoBusTraits>;
    using RAIManipulationDemoInterface = AZ::Interface<RAIManipulationDemoRequests>;

} // namespace RAIManipulationDemo
