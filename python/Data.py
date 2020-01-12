
class Data(object):
    '''
    Data class to store the data saved in the files
    Usually the dictionary contains two keys:
    - values
    - names
    '''

    def __init__(self):
        self.residuals= {}
        self.errors= {}
        self.var= {}
        self.lir= {}
        self.estimated_states= {}
        self.true_states= {}
