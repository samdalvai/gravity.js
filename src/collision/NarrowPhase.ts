/*
 * Portions of this file are derived from Box2D and Phaser Box2D.
 *
 * Copyright (c) 2023 Erin Catto
 * Copyright (c) 2024 Phaser Studio Inc
 * Licensed under the MIT License
 */
import { SETTINGS } from '../core/Constants';
import { RigidBody } from '../core/RigidBody';
import { Vec2 } from '../math/Vec2';
import { CapsuleShape } from '../shapes/CapsuleShape';
import { CircleShape } from '../shapes/CircleShape';
import { PolygonShape } from '../shapes/PolygonShape';
import { ShapeType } from '../shapes/Shape';
import * as Utils from '../utils/Utils';
import { ContactManifold, ContactPoint } from './ContactManifold';

export function detectCollision(bodyA: RigidBody, bodyB: RigidBody): ContactManifold | null {
    const aType = bodyA.shapeType;
    const bType = bodyB.shapeType;

    if (aType === ShapeType.CIRCLE && bType === ShapeType.CIRCLE) {
        return collideCircles(bodyA, bodyB);
    }

    if (aType === ShapeType.CAPSULE && bType === ShapeType.CAPSULE) {
        return collideCapsules(bodyA, bodyB);
    }

    if (aType === ShapeType.CAPSULE && bType === ShapeType.CIRCLE) {
        return collideCapsuleCircle(bodyA, bodyB);
    }

    if (aType === ShapeType.CIRCLE && bType === ShapeType.CAPSULE) {
        return collideCapsuleCircle(bodyB, bodyA);
    }

    if (aType === ShapeType.SEGMENT && bType === ShapeType.CIRCLE) {
        return collideSegmentCircle(bodyA, bodyB);
    }

    if (aType === ShapeType.CIRCLE && bType === ShapeType.SEGMENT) {
        return collideSegmentCircle(bodyB, bodyA);
    }

    const aIsPolygon = isPolygonShape(aType);
    const bIsPolygon = isPolygonShape(bType);

    if (aIsPolygon && bType === ShapeType.CIRCLE) {
        return collidePolygonCircle(bodyA, bodyB);
    }

    if (aType === ShapeType.CIRCLE && bIsPolygon) {
        return collidePolygonCircle(bodyB, bodyA);
    }

    const aIsPolygonLike = isPolygonLikeShape(aType);
    const bIsPolygonLike = isPolygonLikeShape(bType);

    if (aIsPolygonLike && bIsPolygonLike) {
        return collidePolygonLikeBodies(bodyA, bodyB);
    }

    if (aIsPolygonLike && bType === ShapeType.CAPSULE) {
        return collidePolygonLikeAndCapsule(bodyA, bodyB);
    }

    if (aType === ShapeType.CAPSULE && bIsPolygonLike) {
        return collidePolygonLikeAndCapsule(bodyB, bodyA);
    }

    return null;
}

function isPolygonShape(shapeType: ShapeType): boolean {
    return shapeType === ShapeType.BOX || shapeType === ShapeType.POLYGON;
}

function isPolygonLikeShape(shapeType: ShapeType): boolean {
    return isPolygonShape(shapeType) || shapeType === ShapeType.SEGMENT;
}

// TODO: can we avoid calling this method for each collision?
function createCollisionManifold(
    bodyA: RigidBody,
    bodyB: RigidBody,
    normal: Vec2,
    points: ContactPoint[],
): ContactManifold | null {
    if (points.length === 0) {
        return null;
    }

    let depth = 0;

    for (const point of points) {
        depth = Math.max(depth, -point.separation);
    }

    return new ContactManifold(bodyA, bodyB, points, depth, normal, false);
}

function segmentDistance(
    p1X: number,
    p1Y: number,
    q1X: number,
    q1Y: number,
    p2X: number,
    p2Y: number,
    q2X: number,
    q2Y: number,
): {
    fraction1: number;
    fraction2: number;
    distanceSquared: number;
} {
    let fraction1 = 0;
    let fraction2 = 0;

    const d1X = q1X - p1X;
    const d1Y = q1Y - p1Y;
    const d2X = q2X - p2X;
    const d2Y = q2Y - p2Y;
    const rX = p1X - p2X;
    const rY = p1Y - p2Y;
    const dd1 = d1X * d1X + d1Y * d1Y;
    const dd2 = d2X * d2X + d2Y * d2Y;
    const rd2 = rX * d2X + rY * d2Y;
    const rd1 = rX * d1X + rY * d1Y;

    if (dd1 < 0 || dd2 < 0) {
        if (dd1 >= 0) {
            fraction1 = Utils.clamp(-rd1 / dd1, 0.0, 1.0);
            fraction2 = 0.0;
        } else if (dd2 >= 0) {
            fraction1 = 0.0;
            fraction2 = Utils.clamp(rd2 / dd2, 0.0, 1.0);
        }
    } else {
        const d12 = d1X * d2X + d1Y * d2Y;
        const denominator = dd1 * dd2 - d12 * d12;

        let f1 = 0.0;
        if (denominator !== 0.0) {
            f1 = Utils.clamp((d12 * rd2 - rd1 * dd2) / denominator, 0.0, 1.0);
        }

        let f2 = (d12 * f1 + rd2) / dd2;

        if (f2 < 0.0) {
            f2 = 0.0;
            f1 = Utils.clamp(-rd1 / dd1, 0.0, 1.0);
        } else if (f2 > 1.0) {
            f2 = 1.0;
            f1 = Utils.clamp((d12 - rd1) / dd1, 0.0, 1.0);
        }

        fraction1 = f1;
        fraction2 = f2;
    }

    const closest1X = p1X + fraction1 * d1X;
    const closest1Y = p1Y + fraction1 * d1Y;
    const closest2X = p2X + fraction2 * d2X;
    const closest2Y = p2Y + fraction2 * d2Y;
    const dx = closest1X - closest2X;
    const dy = closest1Y - closest2Y;

    return {
        fraction1,
        fraction2,
        distanceSquared: dx * dx + dy * dy,
    };
}

export function collideCircles(bodyA: RigidBody, bodyB: RigidBody): ContactManifold | null {
    const circleA = bodyA.shape as CircleShape;
    const circleB = bodyB.shape as CircleShape;
    const radiusA = circleA.radius;
    const radiusB = circleB.radius;
    const posA = bodyA.position;
    const posB = bodyB.position;

    const ab = posB.subNew(posA);
    const radiusSum = radiusA + radiusB;
    const distSq = ab.magnitudeSquared();

    if (distSq > radiusSum * radiusSum) {
        return null;
    }

    const normal = distSq > 0 ? ab.normalizeNew() : new Vec2(1, 0);
    const distance = ab.magnitude();
    // const penetrationDepth = radiusSum - distance;
    const pointA = posA.addNew(normal.scaleNew(radiusA));
    const pointB = posB.subNew(normal.scaleNew(radiusB));
    // const contactPoint = pointA.addNew(pointB).scaleNew(0.5);

    return createCollisionManifold(bodyA, bodyB, normal, [
        {
            point: pointA.addNew(pointB).scaleNew(0.5),
            separation: distance - radiusSum,
            id: 0,
        },
    ]);

    // TODO: separation is not needed here
    // return new ContactManifold(
    //     bodyA,
    //     bodyB,
    //     [{ point: contactPoint, separation: distance - radiusSum, id: 0 }],
    //     penetrationDepth,
    //     normal,
    //     false,
    // );
}

function collideSegmentRadiusAndCircle(
    bodyA: RigidBody,
    bodyB: RigidBody,
    startA: Vec2,
    endA: Vec2,
    radiusA: number,
): ContactManifold | null {
    const circleB = bodyB.shape as CircleShape;
    const edge = endA.subNew(startA);
    const startProjection = bodyB.position.subNew(startA).dot(edge);
    const endProjection = endA.subNew(bodyB.position).dot(edge);

    let pointA: Vec2;

    if (startProjection < 0.0) {
        pointA = startA;
    } else if (endProjection < 0.0) {
        pointA = endA;
    } else {
        const edgeLengthSquared = edge.dot(edge);
        const t = edgeLengthSquared > 0.0 ? startProjection / edgeLengthSquared : 0.0;
        pointA = startA.addNew(edge.scaleNew(t));
    }

    const fallbackNormal = edge.magnitudeSquared() > 0 ? edge.leftPerpNew().normalizeNew() : new Vec2(1, 0);
    const result = bodyB.position.subNew(pointA).lengthAndNormalize(0, fallbackNormal);
    const separation = result.length - radiusA - circleB.radius;

    if (separation > 0) {
        return null;
    }

    const contactA = pointA.addNew(result.normal.scaleNew(radiusA));
    const contactB = bodyB.position.subNew(result.normal.scaleNew(circleB.radius));

    return createCollisionManifold(bodyA, bodyB, result.normal, [
        {
            point: contactA.lerp(contactB, 0.5),
            separation,
            id: 0,
        },
    ]);
}

export function collidePolygonCircle(bodyA: RigidBody, bodyB: RigidBody): ContactManifold | null {
    const polygonA = bodyA.shape as PolygonShape;
    const circleB = bodyB.shape as CircleShape;
    const vertices = polygonA.worldVertices;
    const normals = polygonA.worldNormals;
    const radius = polygonA.radius + circleB.radius;

    let normalIndex = 0;
    let separation = Number.NEGATIVE_INFINITY;

    for (let i = 0; i < vertices.length; ++i) {
        const candidate = bodyB.position.dotSub(vertices[i], normals[i]);

        if (candidate > separation) {
            separation = candidate;
            normalIndex = i;
        }
    }

    if (separation > radius + 0) {
        return null;
    }

    const vertex1 = vertices[normalIndex];
    const vertex2 = vertices[(normalIndex + 1) % vertices.length];
    const u1 = bodyB.position.subNew(vertex1).dot(vertex2.subNew(vertex1));
    const u2 = bodyB.position.subNew(vertex2).dot(vertex1.subNew(vertex2));

    if (u1 < 0.0 && separation > 0) {
        const delta = bodyB.position.subNew(vertex1);
        const result = delta.lengthAndNormalize(0);
        const vertexSeparation = delta.dot(result.normal);

        if (vertexSeparation > radius + 0) {
            return null;
        }

        const pointA = vertex1.addNew(result.normal.scaleNew(polygonA.radius));
        const pointB = bodyB.position.subNew(result.normal.scaleNew(circleB.radius));

        return createCollisionManifold(bodyA, bodyB, result.normal, [
            {
                point: pointA.lerp(pointB, 0.5),
                separation: vertexSeparation - radius,
                id: 0,
            },
        ]);
    }

    if (u2 < 0.0 && separation > 0) {
        const delta = bodyB.position.subNew(vertex2);
        const result = delta.lengthAndNormalize(0);
        const vertexSeparation = delta.dot(result.normal);

        if (vertexSeparation > radius + 0) {
            return null;
        }

        const pointA = vertex2.addNew(result.normal.scaleNew(polygonA.radius));
        const pointB = bodyB.position.subNew(result.normal.scaleNew(circleB.radius));

        return createCollisionManifold(bodyA, bodyB, result.normal, [
            {
                point: pointA.lerp(pointB, 0.5),
                separation: vertexSeparation - radius,
                id: 0,
            },
        ]);
    }

    const normal = normals[normalIndex];
    const pointA = bodyB.position.addNew(normal.scaleNew(polygonA.radius - bodyB.position.dotSub(vertex1, normal)));
    const pointB = bodyB.position.subNew(normal.scaleNew(circleB.radius));

    return createCollisionManifold(bodyA, bodyB, normal, [
        {
            point: pointA.lerp(pointB, 0.5),
            separation: separation - radius,
            id: 0,
        },
    ]);
}

function clipConvexEdges(
    bodyA: RigidBody,
    bodyB: RigidBody,
    verticesA: readonly Vec2[],
    normalsA: readonly Vec2[],
    radiusA: number,
    verticesB: readonly Vec2[],
    normalsB: readonly Vec2[],
    radiusB: number,
    edgeA: number,
    edgeB: number,
    flip: boolean,
): ContactManifold | null {
    const referenceVertices = flip ? verticesB : verticesA;
    const referenceNormals = flip ? normalsB : normalsA;
    const referenceRadius = flip ? radiusB : radiusA;
    const incidentVertices = flip ? verticesA : verticesB;
    const incidentRadius = flip ? radiusA : radiusB;
    const i11 = flip ? edgeB : edgeA;
    const i12 = (i11 + 1) % referenceVertices.length;
    const i21 = flip ? edgeA : edgeB;
    const i22 = (i21 + 1) % incidentVertices.length;

    const normal = referenceNormals[i11];
    const v11 = referenceVertices[i11];
    const v12 = referenceVertices[i12];
    const v21 = incidentVertices[i21];
    const v22 = incidentVertices[i22];

    const tangent = normal.leftPerpNew();
    const upper1 = v12.dotSub(v11, tangent);
    const upper2 = v21.dotSub(v11, tangent);
    const lower2 = v22.dotSub(v11, tangent);

    const vLower = lower2 < 0.0 && upper2 - lower2 > 0 ? v22.lerp(v21, (0.0 - lower2) / (upper2 - lower2)) : v22;
    const vUpper = upper2 > upper1 && upper2 - lower2 > 0 ? v22.lerp(v21, (upper1 - lower2) / (upper2 - lower2)) : v21;

    const separationLower = vLower.dotSub(v11, normal);
    const separationUpper = vUpper.dotSub(v11, normal);
    const radius = radiusA + radiusB;
    const pointLower = vLower.addNew(normal.scaleNew(0.5 * (referenceRadius - incidentRadius - separationLower)));
    const pointUpper = vUpper.addNew(normal.scaleNew(0.5 * (referenceRadius - incidentRadius - separationUpper)));

    const points: ContactPoint[] = [
        {
            point: pointLower,
            separation: separationLower - radius,
            id: Utils.makeId(i11, i22),
        },
        {
            point: pointUpper,
            separation: separationUpper - radius,
            id: Utils.makeId(i12, i21),
        },
    ];

    if (flip) {
        [points[0], points[1]] = [points[1], points[0]];
    }

    const filteredPoints: ContactPoint[] = [];

    // TODO: is this needed?
    for (let i = 0; i < points.length; i++) {
        const p = points[i];
        if (p.separation <= 0) {
            filteredPoints.push(p);
        }
    }

    if (
        filteredPoints.length === 2 &&
        filteredPoints[0].point.distanceSquared(filteredPoints[1].point) <= SETTINGS.contactMergeThreshold
    ) {
        filteredPoints.length = 1;
    }

    return createCollisionManifold(bodyA, bodyB, flip ? normal.negateNew() : normal, filteredPoints);
}

function findMaxSeparation(
    verticesA: readonly Vec2[],
    normalsA: readonly Vec2[],
    verticesB: readonly Vec2[],
): { edgeIndex: number; maxSeparation: number } {
    let bestIndex = 0;
    let maxSeparation = Number.NEGATIVE_INFINITY;

    for (let i = 0; i < verticesA.length; ++i) {
        const normal = normalsA[i];
        const vertex = verticesA[i];
        let separation = Number.POSITIVE_INFINITY;

        for (let j = 0; j < verticesB.length; ++j) {
            const candidate = verticesB[j].dotSub(vertex, normal);
            if (candidate < separation) {
                separation = candidate;
            }
        }

        if (separation > maxSeparation) {
            maxSeparation = separation;
            bestIndex = i;
        }
    }

    return { edgeIndex: bestIndex, maxSeparation };
}

function collideConvexPolygons(
    bodyA: RigidBody,
    bodyB: RigidBody,
    verticesA: readonly Vec2[],
    normalsA: readonly Vec2[],
    radiusA: number,
    verticesB: readonly Vec2[],
    normalsB: readonly Vec2[],
    radiusB: number,
): ContactManifold | null {
    const { edgeIndex: initialEdgeA, maxSeparation: separationA } = findMaxSeparation(verticesA, normalsA, verticesB);
    const { edgeIndex: initialEdgeB, maxSeparation: separationB } = findMaxSeparation(verticesB, normalsB, verticesA);
    const radius = radiusA + radiusB;

    if (separationA > radius || separationB > radius) {
        return null;
    }

    let edgeA = initialEdgeA;
    let edgeB = initialEdgeB;
    let flip = false;

    if (separationA >= separationB) {
        const searchDirection = normalsA[edgeA];
        let minDot = Number.MAX_VALUE;

        for (let i = 0; i < normalsB.length; ++i) {
            const dot = searchDirection.dot(normalsB[i]);
            if (dot < minDot) {
                minDot = dot;
                edgeB = i;
            }
        }
    } else {
        flip = true;

        const searchDirection = normalsB[edgeB];
        let minDot = Number.MAX_VALUE;
        edgeA = 0;

        for (let i = 0; i < normalsA.length; ++i) {
            const dot = searchDirection.dot(normalsA[i]);
            if (dot < minDot) {
                minDot = dot;
                edgeA = i;
            }
        }
    }

    if (separationA > 0 || separationB > 0) {
        const i11 = edgeA;
        const i12 = (edgeA + 1) % verticesA.length;
        const i21 = edgeB;
        const i22 = (edgeB + 1) % verticesB.length;
        const v11 = verticesA[i11];
        const v12 = verticesA[i12];
        const v21 = verticesB[i21];
        const v22 = verticesB[i22];
        const result = segmentDistance(v11.x, v11.y, v12.x, v12.y, v21.x, v21.y, v22.x, v22.y);

        if (result.fraction1 === 0.0 && result.fraction2 === 0.0) {
            const delta = v21.subNew(v11);
            const distance = delta.magnitude();
            if (distance > radius) {
                return null;
            }

            const normal = distance > 0 ? delta.divNew(distance) : normalsA[edgeA].copy();
            const pointA = v11.addNew(normal.scaleNew(radiusA));
            const pointB = v21.subNew(normal.scaleNew(radiusB));

            return createCollisionManifold(bodyA, bodyB, normal, [
                {
                    point: pointA.lerp(pointB, 0.5),
                    separation: distance - radius,
                    id: Utils.makeId(i11, i21),
                },
            ]);
        }

        if (result.fraction1 === 0.0 && result.fraction2 === 1.0) {
            const delta = v22.subNew(v11);
            const distance = delta.magnitude();
            if (distance > radius) {
                return null;
            }

            const normal = distance > 0 ? delta.divNew(distance) : normalsA[edgeA].copy();
            const pointA = v11.addNew(normal.scaleNew(radiusA));
            const pointB = v22.subNew(normal.scaleNew(radiusB));

            return createCollisionManifold(bodyA, bodyB, normal, [
                {
                    point: pointA.lerp(pointB, 0.5),
                    separation: distance - radius,
                    id: Utils.makeId(i11, i22),
                },
            ]);
        }

        if (result.fraction1 === 1.0 && result.fraction2 === 0.0) {
            const delta = v21.subNew(v12);
            const distance = delta.magnitude();
            if (distance > radius) {
                return null;
            }

            const normal = distance > 0 ? delta.divNew(distance) : normalsA[edgeA].copy();
            const pointA = v12.addNew(normal.scaleNew(radiusA));
            const pointB = v21.subNew(normal.scaleNew(radiusB));

            return createCollisionManifold(bodyA, bodyB, normal, [
                {
                    point: pointA.lerp(pointB, 0.5),
                    separation: distance - radius,
                    id: Utils.makeId(i12, i21),
                },
            ]);
        }

        if (result.fraction1 === 1.0 && result.fraction2 === 1.0) {
            const delta = v22.subNew(v12);
            const distance = delta.magnitude();
            if (distance > radius) {
                return null;
            }

            const normal = distance > 0 ? delta.divNew(distance) : normalsA[edgeA].copy();
            const pointA = v12.addNew(normal.scaleNew(radiusA));
            const pointB = v22.subNew(normal.scaleNew(radiusB));

            return createCollisionManifold(bodyA, bodyB, normal, [
                {
                    point: pointA.lerp(pointB, 0.5),
                    separation: distance - radius,
                    id: Utils.makeId(i12, i22),
                },
            ]);
        }
    }

    return clipConvexEdges(
        bodyA,
        bodyB,
        verticesA,
        normalsA,
        radiusA,
        verticesB,
        normalsB,
        radiusB,
        edgeA,
        edgeB,
        flip,
    );
}

function collidePolygonLikeBodies(bodyA: RigidBody, bodyB: RigidBody): ContactManifold | null {
    const shapeA = bodyA.shape as PolygonShape;
    const shapeB = bodyB.shape as PolygonShape;

    return collideConvexPolygons(
        bodyA,
        bodyB,
        shapeA.worldVertices,
        shapeA.worldNormals,
        shapeA.radius,
        shapeB.worldVertices,
        shapeB.worldNormals,
        shapeB.radius,
    );
}

function collideSegmentRadiusPairs(
    bodyA: RigidBody,
    bodyB: RigidBody,
    p1: Vec2,
    q1: Vec2,
    radiusA: number,
    p2: Vec2,
    q2: Vec2,
    radiusB: number,
): ContactManifold | null {
    const d1 = q1.subNew(p1);
    const d2 = q2.subNew(p2);
    const result = segmentDistance(p1.x, p1.y, q1.x, q1.y, p2.x, p2.y, q2.x, q2.y);
    const f1 = result.fraction1;
    const f2 = result.fraction2;
    const closest1 = p1.addNew(d1.scaleNew(f1));
    const closest2 = p2.addNew(d2.scaleNew(f2));
    const radius = radiusA + radiusB;
    const maxDistance = radius + 0;

    if (result.distanceSquared > maxDistance * maxDistance) {
        return null;
    }

    const length1 = d1.magnitude();
    const u1 = length1 > 0 ? d1.divNew(length1) : new Vec2(0, 1);
    const length2 = d2.magnitude();
    const u2 = length2 > 0 ? d2.divNew(length2) : new Vec2(0, 1);
    const fp2 = p2.subNew(p1).dot(u1);
    const fq2 = q2.subNew(p1).dot(u1);
    const outsideA = (fp2 <= 0.0 && fq2 <= 0.0) || (fp2 >= length1 && fq2 >= length1);
    const fp1 = p1.subNew(p2).dot(u2);
    const fq1 = q1.subNew(p2).dot(u2);
    const outsideB = (fp1 <= 0.0 && fq1 <= 0.0) || (fp1 >= length2 && fq1 >= length2);
    const closestDistance = Math.sqrt(result.distanceSquared);

    if (!outsideA && !outsideB) {
        let normalA = u1.leftPerpNew();
        const ssA1 = p2.subNew(p1).dot(normalA);
        const ssA2 = q2.subNew(p1).dot(normalA);
        const sAPositive = Math.min(ssA1, ssA2);
        const sANegative = Math.max(-ssA1, -ssA2);
        const separationA = sAPositive > sANegative ? sAPositive : sANegative;
        if (sAPositive <= sANegative) {
            normalA = normalA.negateNew();
        }

        let normalB = u2.leftPerpNew();
        const ssB1 = p1.subNew(p2).dot(normalB);
        const ssB2 = q1.subNew(p2).dot(normalB);
        const sBPositive = Math.min(ssB1, ssB2);
        const sBNegative = Math.max(-ssB1, -ssB2);
        const separationB = sBPositive > sBNegative ? sBPositive : sBNegative;
        if (sBPositive <= sBNegative) {
            normalB = normalB.negateNew();
        }

        if (separationA >= separationB) {
            let cp = p2.copy();
            let cq = q2.copy();

            if (fp2 < 0.0 && fq2 > 0.0) {
                cp = p2.lerp(q2, (0.0 - fp2) / (fq2 - fp2));
            } else if (fq2 < 0.0 && fp2 > 0.0) {
                cq = q2.lerp(p2, (0.0 - fq2) / (fp2 - fq2));
            }

            if (fp2 > length1 && fq2 < length1) {
                cp = p2.lerp(q2, (fp2 - length1) / (fp2 - fq2));
            } else if (fq2 > length1 && fp2 < length1) {
                cq = q2.lerp(p2, (fq2 - length1) / (fq2 - fp2));
            }

            const sp = cp.subNew(p1).dot(normalA);
            const sq = cq.subNew(p1).dot(normalA);

            if (sp <= closestDistance || sq <= closestDistance) {
                return createCollisionManifold(bodyA, bodyB, normalA, [
                    {
                        point: cp.addNew(normalA.scaleNew(0.5 * (radiusA - radiusB - sp))),
                        separation: sp - radius,
                        id: Utils.makeId(0, 0),
                    },
                    {
                        point: cq.addNew(normalA.scaleNew(0.5 * (radiusA - radiusB - sq))),
                        separation: sq - radius,
                        id: Utils.makeId(0, 1),
                    },
                ]);
            }
        } else {
            const manifoldNormal = normalB.negateNew();
            let cp = p1.copy();
            let cq = q1.copy();

            if (fp1 < 0.0 && fq1 > 0.0) {
                cp = p1.lerp(q1, (0.0 - fp1) / (fq1 - fp1));
            } else if (fq1 < 0.0 && fp1 > 0.0) {
                cq = q1.lerp(p1, (0.0 - fq1) / (fp1 - fq1));
            }

            if (fp1 > length2 && fq1 < length2) {
                cp = p1.lerp(q1, (fp1 - length2) / (fp1 - fq1));
            } else if (fq1 > length2 && fp1 < length2) {
                cq = q1.lerp(p1, (fq1 - length2) / (fq1 - fp1));
            }

            const sp = cp.subNew(p2).dot(normalB);
            const sq = cq.subNew(p2).dot(normalB);

            if (sp <= closestDistance || sq <= closestDistance) {
                return createCollisionManifold(bodyA, bodyB, manifoldNormal, [
                    {
                        point: cp.addNew(normalB.scaleNew(0.5 * (radiusB - radiusA - sp))),
                        separation: sp - radius,
                        id: Utils.makeId(0, 0),
                    },
                    {
                        point: cq.addNew(normalB.scaleNew(0.5 * (radiusB - radiusA - sq))),
                        separation: sq - radius,
                        id: Utils.makeId(1, 0),
                    },
                ]);
            }
        }
    }

    const fallbackNormal = u1.leftPerpNew().normalizeNew();
    const delta = closest2.subNew(closest1);
    const closestResult = delta.lengthAndNormalize(0, fallbackNormal);
    const pointA = closest1.addNew(closestResult.normal.scaleNew(radiusA));
    const pointB = closest2.subNew(closestResult.normal.scaleNew(radiusB));
    const idA = f1 === 0.0 ? 0 : 1;
    const idB = f2 === 0.0 ? 0 : 1;

    return createCollisionManifold(bodyA, bodyB, closestResult.normal, [
        {
            point: pointA.lerp(pointB, 0.5),
            separation: closestResult.length - radius,
            id: Utils.makeId(idA, idB),
        },
    ]);
}

export function collideCapsuleCircle(bodyA: RigidBody, bodyB: RigidBody): ContactManifold | null {
    const capsuleA = bodyA.shape as CapsuleShape;

    return collideSegmentRadiusAndCircle(bodyA, bodyB, capsuleA.worldCenter1, capsuleA.worldCenter2, capsuleA.radius);
}

export function collideCapsules(bodyA: RigidBody, bodyB: RigidBody): ContactManifold | null {
    const capsuleA = bodyA.shape as CapsuleShape;
    const capsuleB = bodyB.shape as CapsuleShape;

    return collideSegmentRadiusPairs(
        bodyA,
        bodyB,
        capsuleA.worldCenter1,
        capsuleA.worldCenter2,
        capsuleA.radius,
        capsuleB.worldCenter1,
        capsuleB.worldCenter2,
        capsuleB.radius,
    );
}

function collideSegmentCircle(bodyA: RigidBody, bodyB: RigidBody): ContactManifold | null {
    const segmentA = bodyA.shape as PolygonShape;

    return collideSegmentRadiusAndCircle(bodyA, bodyB, segmentA.worldVertices[0], segmentA.worldVertices[1], 0);
}

function collidePolygonLikeAndCapsule(bodyA: RigidBody, bodyB: RigidBody): ContactManifold | null {
    const shapeA = bodyA.shape as PolygonShape;
    const capsuleB = bodyB.shape as CapsuleShape;
    const capsuleVertices = [capsuleB.worldCenter1, capsuleB.worldCenter2];
    const axis = capsuleB.worldCenter1.subNew(capsuleB.worldCenter2);
    const normal = axis.magnitudeSquared() > 0 ? axis.rightPerpNew().normalizeNew() : new Vec2(1, 0);
    const capsuleNormals = [normal, normal.negateNew()];

    return collideConvexPolygons(
        bodyA,
        bodyB,
        shapeA.worldVertices,
        shapeA.worldNormals,
        shapeA.radius,
        capsuleVertices,
        capsuleNormals,
        capsuleB.radius,
    );
}
