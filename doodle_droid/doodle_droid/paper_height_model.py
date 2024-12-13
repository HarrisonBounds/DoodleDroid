class PaperHeightModel(object):
    def __init__(self, **kwargs):
        raise NotImplementedError()
    def update_model(self, **kwargs):
        raise NotImplementedError()
    def export(self):
        raise NotImplementedError()
    def get_paper_height(self, x, y):
        raise NotImplementedError()
    
    
class PlanePaperHeightModel(PaperHeightModel):
    """
    see wikipedia for plane equations.
    https://en.wikipedia.org/wiki/Euclidean_planes_in_three-dimensional_space#Point%E2%80%93normal_form_and_general_form_of_the_equation_of_a_plane


    """
    def __init__(self, a=0, b=0, c=1, d=0):
        self.a = a
        self.b = b
        self.c = c
        self.d = d
        self._validate_model()

    def _validate_model(self):
        assert self.c != 0, "c cannot be zero"

    def update_model(self, a, b, c, d):
        self._validate_model()
        self.a = a
        self.b = b
        self.c = c
        self.d = d

    def export(self):
        return {'a': self.a, 'b': self.b, 'c': self.c, 'd': self.d}

    def get_paper_height(self, x, y):
        z = -(self.a * x + self.b * y + self.d) / self.c
        return z
