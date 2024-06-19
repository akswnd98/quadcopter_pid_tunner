class PhysicsStepPreparer:
  def update (self):
    pass

class ContainerPhysicsStepPreparer (PhysicsStepPreparer):
  def __init__ (self, preparers: list[PhysicsStepPreparer]):
    self.preparers = preparers

  def update (self):
    for preparer in self.preparers:
      preparer.update()
