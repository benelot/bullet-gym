import unittest
import Trainer
import KerasDQNAgent
import Detached2DCartPolev0Env


class KerasDQNAgentTest(unittest.TestCase):


    def setUp(self):
        trainer = Trainer()
        agent = KerasDQNAgent()
        env = Detached2DCartPolev0Env()


    def tearDown(self):
        pass


    def testDQN(self):
        pass


if __name__ == "__main__":
    #import sys;sys.argv = ['', 'Test.testName']
    unittest.main()