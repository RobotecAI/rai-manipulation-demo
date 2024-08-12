
#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/Module/Module.h>

#include "RAIManipulationDemoSystemComponent.h"

namespace RAIManipulationDemo
{
    class RAIManipulationDemoModule
        : public AZ::Module
    {
    public:
        AZ_RTTI(RAIManipulationDemoModule, "{C28F2350-CF82-4848-B392-A9B94B5B681B}", AZ::Module);
        AZ_CLASS_ALLOCATOR(RAIManipulationDemoModule, AZ::SystemAllocator, 0);

        RAIManipulationDemoModule()
            : AZ::Module()
        {
            // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
            m_descriptors.insert(m_descriptors.end(), {
                RAIManipulationDemoSystemComponent::CreateDescriptor(),
            });
        }

        /**
         * Add required SystemComponents to the SystemEntity.
         */
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList{
                azrtti_typeid<RAIManipulationDemoSystemComponent>(),
            };
        }
    };
}// namespace RAIManipulationDemo

AZ_DECLARE_MODULE_CLASS(Gem_RAIManipulationDemo, RAIManipulationDemo::RAIManipulationDemoModule)
