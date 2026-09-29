import unittest

MODULE = None  # DO NOT USE OR TOUCH THIS VARIABLE


"""You may change all of these Test__ Classes! You won't submit this file, but it will help you test your code!"""


class TestQ01(unittest.TestCase):
    def test_1(self):
        self.assertEqual(MODULE.q1(1, 1), 2)

    def test_2(self):
        self.assertEqual(MODULE.q1(1, 2), 3)

    def test_3(self):
        self.assertEqual(MODULE.q1(2, 1), 3)


class TestQ02(unittest.TestCase):
    def test_1(self):
        self.assertEqual(MODULE.q2(3, 4), 12)


class TestQ03(unittest.TestCase):
    def test_1(self):
        self.assertEqual(MODULE.q3(3, 2), 9)


class TestQ04(unittest.TestCase):
    def test_1(self):
        self.assertEqual(MODULE.q4(31), (62, 28, 3))


class TestQ05(unittest.TestCase):
    def test_1(self):
        self.assertEqual(MODULE.q5(1, 4), 1 + 2 + 3 + 4)


class TestQ06(unittest.TestCase):
    def test_1(self):
        self.assertEqual(MODULE.q6(10), 825)


class TestQ07(unittest.TestCase):
    def test_1(self):
        self.assertEqual(MODULE.q7(4), [0, -3, 6, -9])


class TestQ08(unittest.TestCase):
    def test_1(self):
        self.assertEqual(
            MODULE.q8([10, 9, 8, 7, 6, 10, 4, 3, 10, 1]), [0, 5, 8])

    def test_2(self):
        self.assertEqual(
            MODULE.q8([]), [])

    def test_3(self):
        self.assertEqual(MODULE.q8([10]), [0])


class TestQ09(unittest.TestCase):
    def test_1(self):
        res, gpa = MODULE.q9(
            {'Calc 2': 2.3, 'Physics': 3.4, 'Algorithms': 2.1})
        ans1, ans2 = (['Calc 2', 'Physics', 'Algorithms'], 2.6)
        self.assertEqual(res, ans1)
        self.assertAlmostEqual(gpa, ans2)


class TestQ10(unittest.TestCase):
    def test_1(self):
        self.assertAlmostEqual(MODULE.q10(6.25), 5)

    def test_2(self):
        self.assertAlmostEqual(MODULE.q10(4), 0)

    def test_3(self):
        self.assertAlmostEqual(MODULE.q10(256), 0)


class TestQ11(unittest.TestCase):
    def test_1(self):
        x, y, z = MODULE.q11(1, 0, 0)
        self.assertAlmostEqual(x, 1)
        self.assertAlmostEqual(y, 0)
        self.assertAlmostEqual(z, 0)

    def test_2(self):
        x, y, z = MODULE.q11(3, 4, 0)
        self.assertAlmostEqual(x, 0.6)
        self.assertAlmostEqual(y, 0.8)
        self.assertAlmostEqual(z, 0.0)

    def test_3(self):
        x, y, z = MODULE.q11(0, 100, 0)
        self.assertAlmostEqual(x, 0)
        self.assertAlmostEqual(y, 1)
        self.assertAlmostEqual(z, 0)

    def test_4(self):
        x, y, z = MODULE.q11(4, 5, 6)
        self.assertAlmostEqual(x, 0.4558423058385518)
        self.assertAlmostEqual(y, 0.5698028822981898)
        self.assertAlmostEqual(z, 0.6837634587578276)


class TestQ12(unittest.TestCase):
    def test_1(self):
        x, y = MODULE.q12(5, 0)
        self.assertAlmostEqual(x, 5)
        self.assertAlmostEqual(y, 0)

    def test_2(self):
        x, y = MODULE.q12(5, 90)
        self.assertAlmostEqual(x, 0)
        self.assertAlmostEqual(y, 5)


class TestQ13(unittest.TestCase):
    def test_1(self):
        data = [
            [1, 2, 3],
            [1, 1, 1]
        ]
        text = MODULE.q13(data)
        ans = "1,3,2,2\n1,1,1,1\n"
        self.assertEqual(text, ans)


class TestQ14(unittest.TestCase):
    def test_1(self):
        text = "1,2,3\n4,5,6"
        ans = [
            [1, 2, 3],
            [4, 5, 6]
        ]
        output = MODULE.q14(text)
        self.assertEqual(output, ans)


"""
==============================================================================
BEGINNING OF THE TEST RUNNING CODE. DO NOT TOUCH ANYTHING BELOW THIS COMMMENT.
IT PROBABLY WON'T HELP YOU TO CHANGE IT.























==============================================================================
"""

TEST_CASE = None


class CustomTestRunner(unittest.TextTestRunner):
    def _makeResult(self) -> unittest.result.TestResult:
        return CustomTestResult(self.stream, self.descriptions, self.verbosity)


class CustomTestResult(unittest.TextTestResult):
    def addSuccess(self, test):
        super(CustomTestResult, self).addSuccess(test)

        class_name = test.__class__.__name__
        test_name = test._testMethodName

        if class_name not in TEST_CASE:
            TEST_CASE[class_name] = {'fail': [], 'success': []}

        TEST_CASE[class_name]['success'].append(test_name)

    def addFailure(self, test, err):
        super(CustomTestResult, self).addFailure(test, err)

        class_name = test.__class__.__name__
        test_name = test._testMethodName

        if class_name not in TEST_CASE:
            TEST_CASE[class_name] = {'fail': [], 'success': []}
        e = err[1] if type(err) == tuple else err
        TEST_CASE[class_name]['fail'].append((test_name, str(e).replace('\n', ' ')))

    def addError(self, test, err):
        super(CustomTestResult, self).addError(test, err)

        class_name = test.__class__.__name__
        test_name = test._testMethodName

        if class_name not in TEST_CASE:
            TEST_CASE[class_name] = {'fail': [], 'success': []}

        e = err[1] if type(err) == tuple else err
        TEST_CASE[class_name]['fail'].append((test_name, e))


if __name__ == '__main__':
    import os
    import sys
    import importlib
    modules = []
    assignment_files = []
    if MODULE is None:

        def isassignment(x): return x[-3:] == '.py' and x[:10] == 'assignment'
        def removepy(x): return x[:-3] if x[-3:] == '.py' else x

        assignment_files += list(filter(isassignment, os.listdir()))
    else:
        assignment_files.append(str(MODULE))

    for a in assignment_files:
        module = importlib.import_module(removepy(a))
        if hasattr(module, "STUDENT_NAME") and hasattr(module, "STUDENT_ID"):
            modules.append(module)

    for module in modules:
        TEST_CASE = {}
        MODULE = module
        unittest.main(exit=False, testRunner=CustomTestRunner)

        print(f"For {module.__name__}, the results are:")
        for key, data in TEST_CASE.items():
            succ = len(data['success'])
            fail = len(data['fail'])
            print(f"{key} - {succ} succeeded, {fail} failed")

        import os

        if "results" not in os.listdir():
            os.mkdir("results")
        elif not os.path.isdir("results"):
            continue

        with open(f"results/{module.__name__}.csv", "w", newline="") as f:
            import csv
            writer = csv.writer(f)
            writer.writerow(
                ['Problem Name', 'Correct', 'Incorrect', 'Total Test Cases', 'Percentage Correct', 'Failure Causes'])
            for key, data in TEST_CASE.items():
                succ = len(data['success'])
                fail = len(data['fail'])
                messages = [f"{name}: {msg}" for name, msg in data['fail']]
                writer.writerow([key, succ, fail, succ +
                                fail, succ/(succ+fail)*100, " ||| ".join(messages)])
    input("\n\n>>> Press the 'Enter' key to end this testing program...")
