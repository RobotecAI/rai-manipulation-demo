
#pragma once

#include <AzCore/Component/Component.h>

#include <RAIManipulationDemo/RAIManipulationDemoBus.h>

namespace RAIManipulationDemo
{
    class RAIManipulationDemoSystemComponent
        : public AZ::Component
        , protected RAIManipulationDemoRequestBus::Handler
    {
    public:
        AZ_COMPONENT_DECL(RAIManipulationDemoSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        RAIManipulationDemoSystemComponent();
        ~RAIManipulationDemoSystemComponent();

    protected:
        ////////////////////////////////////////////////////////////////////////
        // RAIManipulationDemoRequestBus interface implementation

        ////////////////////////////////////////////////////////////////////////

        ////////////////////////////////////////////////////////////////////////
        // AZ::Component interface implementation
        void Init() override;
        void Activate() override;
        void Deactivate() override;
        ////////////////////////////////////////////////////////////////////////
    };
}
