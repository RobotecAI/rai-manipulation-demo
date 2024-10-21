
#pragma once

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>

namespace RAIManipulationDemo
{
    class RAIManipulationDemoRequests
    {
    public:
        AZ_RTTI(RAIManipulationDemoRequests, "{61930F09-1C70-4B6D-A05C-9ACDC649DCE1}");
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
