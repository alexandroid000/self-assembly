module Main where

--main :: IO ()
main = do
    let a = mytake 10 [1..]
    let b = rev a
    let c = inclist b
    let d = inclist' b
    return d

-- mytake
-- don't forget to put the type declaration or you will lose points!
--JunWu
mytake :: Int -> [a] -> [a]
mytake 0 _ = []
mytake n [] = []
mytake n (x:xs)
    | n < 0 = error "can't take negative elements"
    | otherwise = x:(mytake (n-1) xs)

-- mydrop
-- don't forget to put the type declaration or you will lose points!
mydrop = undefined

-- rev
-- don't forget to put the type declaration or you will lose points!
--JunWu
rev :: [a] -> [a]
rev [] = []
rev (x:xs) = (rev xs)++[x]

-- app
-- don't forget to put the type declaration or you will lose points!
app = undefined

-- inclist
-- don't forget to put the type declaration or you will lose points!
--Chris
inclist :: (Num a) => [a] -> [a]
inclist [] = []
inclist (x:xs) =  (x+1):(inclist xs)

-- sumlist
-- don't forget to put the type declaration or you will lose points!
--Justin
mysumlist :: Num a => [a] -> a
mysumlist [] = error "Can't Sum an Empty List"
mysumlist [x] = x
mysumlist (x:xs) = x + (mysumlist xs)

-- myzip
-- don't forget to put the type declaration or you will lose points!
myzip = undefined

-- addpairs
-- don't forget to put the type declaration or you will lose points!
-- John
addpairs = undefined

-- ones
-- don't forget to put the type declaration or you will lose points!
ones = undefined

-- nats
-- don't forget to put the type declaration or you will lose points!
nats = undefined

-- fib
-- don't forget to put the type declaration or you will lose points!
-- John
-- todo: optimize
fib :: Int -> [Int]
fib 0 = [0]
fib 1 = [0,1]
fib n =
    let fibn1 = fib (n-1)
    in
    (fib (n-1)) ++ [last (fib (n-1)) + last (fib (n-2))]

-- add
-- don't forget to put the type declaration or you will lose points!
add = undefined

-- union
-- don't forget to put the type declaration or you will lose points!
-- Austin
union = undefined

-- intersect
-- don't forget to put the type declaration or you will lose points!
-- Austin
intersect = undefined

-- powerset
-- don't forget to put the type declaration or you will lose points!
powerset = undefined

-- inclist'
-- don't forget to put the type declaration or you will lose points!
-- Chris
inclist' :: (Num a) => [a] -> [a]
inclist' x = map (+1) x

-- sumlist'
-- don't forget to put the type declaration or you will lose points!
-- Justin
sumlist' :: Num a => [a] -> a
sumlist' = foldr (+) 0

data List a = Cons a (List a)
            | Nil
  deriving (Show, Eq)

data Exp = IntExp Integer
         | PlusExp [Exp]
         | MultExp [Exp]
  deriving (Show, Eq)

-- list2cons
-- don't forget to put the type declaration or you will lose points!
list2cons = undefined

-- cons2list
-- don't forget to put the type declaration or you will lose points!
cons2list = undefined

-- eval
-- don't forget to put the type declaration or you will lose points!
eval = undefined

-- list2cons'
-- don't forget to put the type declaration or you will lose points!
list2cons' = undefined

-- BinTree
-- Li

data BinTree a = Node a (BinTree a) (BinTree a) | Leaf a
                deriving (Show)

myTree = Node 5 (Leaf 2) (Leaf 4)

-- Li
sumTree :: Num a => BinTree a -> a
sumTree (Leaf val) = val
sumTree (Node val left right) = val + (sumTree left) + (sumTree right)

-- SimpVal

-- liftIntOp
-- don't forget to put the type declaration or you will lose points!
liftIntOp = undefined
