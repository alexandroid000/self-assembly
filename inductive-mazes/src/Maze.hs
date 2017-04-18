-- | Code for generating a maze from a randomized graph traversal. A
--   maze is just represented by a list of the walls to draw.
-- 
--   This code was written for my blog post on generating mazes:
--   <http://jelv.is/blog/Generating Mazes with Inductive Graphs/>
module Maze where

import           DFS

import           Control.Monad        (liftM)
import           Control.Monad.Random (MonadRandom)

import qualified Data.Graph.Inductive as Graph
import           Data.Graph.Inductive (Gr)
import qualified Data.List            as List
import           Data.List            ((\\))

-- | Since we're starting with a grid, we only have two possible
--   orientations for walls.
data Orientation = Horizontal | Vertical deriving (Show, Eq)

-- | A wall is positioned relative to the cell that is *below* or *to
--   the right* of it.
data Wall = Wall (Int, Int) Orientation deriving (Eq)

instance Show Wall where
  show (Wall (x, y) Horizontal) = show (x, y) ++ " —"
  show (Wall (x, y) Vertical)   = show (x, y) ++ " |"

-- | We start with a graph representing a graph with nodes as cells
--   and edges as walls.
type Grid = Gr () Wall

-- | Generate a graph representing an n × m grid.
grid :: Int -> Int -> Grid
grid width height = Graph.mkGraph nodes edges
  where nodes = [(node, ()) | node <- [0..width * height - 1]]
        edges = [(n, n', wall n Vertical) |
                 (n, _) <- nodes,
                 (n', _) <- nodes,
                 n - n' == 1 && n `mod` width /= 0 ]
             ++ [(n, n', wall n Horizontal) |
                 (n,_) <- nodes,
                 (n',_) <- nodes,
                 n - n' == width ]
        wall n = let (y, x) = n `divMod` width in Wall (x, y)

complete ::  Int -> Gr String String
complete n = Graph.mkGraph nodes edges  
  where nodes = [(node, "a") | node <- [0..n-1]]
        edges = [(n, n', "b") |
                 (n, _) <- nodes,
                 (n', _) <- nodes,
                 n - n' /= 0]

mkLabels :: [String]
mkLabels =
    let alphabet = ['a'..'z']
        alpha_strings = map (\a -> [a]) alphabet :: [String]
        mkNext :: [String] -> [String]
        mkNext letts = mconcat
                [map (\l -> a:l) letts | a <- alphabet]
    in mconcat $ iterate mkNext alpha_strings

soup :: Int -> Int -> Gr String ()
soup n k = Graph.mkGraph nodes edges
  where m = n `div` k
        label_set = take k mkLabels
        labels = concat (map (replicate m) label_set)
                 ++ (take (n `mod` k) label_set)
        nodes = [(id, label)
                | (id, label) <- zip [1..] labels]
        edges = []

-- | Generates the random edge traversal of an n × m grid.
generate :: MonadRandom m => Int -> Int -> m [Graph.LEdge Wall]
generate width height =
  liftM (Graph.labEdges graph \\) $ edfsR (ghead graph) graph
  where graph = grid width height                    

-- | Look up an edge label in the given graph. I don't know why this
--   isn't in the standard fgl API!
edgeLabel :: Gr a b -> Graph.Edge -> b
edgeLabel graph (n, n') = label
  where edges = Graph.labEdges graph
        Just (_, _, label) = List.find (\ (a, b, l) -> a == n && b == n') edges

-- | Generates a random n × m maze.
maze :: MonadRandom m => Int -> Int -> m [Wall]
maze width height = liftM (map (\(_, _, wall) -> wall)) $ generate width height
